/**
 * FoxgloveConnection —— ws-protocol 客户端封装（ROS 1 版）
 *
 * 与 robviz（ROS 2 版）的差异，全部来自 ros1_foxglove_bridge 的实测协议：
 *   - channel encoding 是 "ros1"（不是 "cdr"）
 *   - schema 是 ROS 1 的 .msg 定义文本，用 parse(schema) 解析（不传选项即 ROS 1）
 *   - 序列化用 @foxglove/rosmsg-serialization（ROS 1 原生布局）
 *   - advertise 不设 schemaEncoding（bridge 自己也不设）
 *   - 空请求服务（如 Trigger）序列化后是 0 字节，没有 CDR 头
 *
 * 零 DOM 依赖：浏览器 Worker 与 Node 端（测试/CLI）通用。
 */
import { parse } from "@foxglove/rosmsg";
import { MessageReader, MessageWriter } from "@foxglove/rosmsg-serialization";
import { FoxgloveClient } from "@foxglove/ws-protocol";
import type { Channel, IWebSocket, Service } from "@foxglove/ws-protocol";

/** ROS 1 bridge 只认这一种编码 */
export const ROS1_ENCODING = "ros1";

export type ConnectionState = "connecting" | "open" | "closed";

export interface TopicInfo {
  topic: string;
  schemaName: string;
  schema: string;
}

export interface ServiceInfo {
  name: string;
  type: string;
  requestSchema: string;
  responseSchema: string;
}

export type MessageHandler = (message: unknown, channel: Channel) => void;

export interface MessageMeta {
  topic: string;
  bytes: number;
  decodeMs: number;
  serverTimeNs: bigint;
}

export interface FoxgloveConnectionOptions {
  /** Node 环境注入 ws 包的 WebSocket；浏览器默认用原生 WebSocket */
  createWebSocket?: (url: string, protocols: string[]) => IWebSocket;
  /** 重连退避基数（ms），0 表示不重连 */
  reconnectMs?: number;
  /** 半开连接看门狗：有订阅但超过该时长无服务端消息则判定卡死并重连。0 关闭 */
  staleTimeoutMs?: number;
  onStateChange?: (state: ConnectionState) => void;
  onChannels?: (topics: TopicInfo[]) => void;
  onServices?: (services: ServiceInfo[]) => void;
  onError?: (error: Error) => void;
  onMessageMeta?: (meta: MessageMeta) => void;
}

const BACKOFF_FACTOR = 1.7;
const BACKOFF_MAX_MS = 10_000;
const BACKOFF_JITTER = 0.3;

export class FoxgloveConnection {
  #url: string;
  #opts: FoxgloveConnectionOptions;
  #client: FoxgloveClient | undefined;
  #closed = false;
  #reconnectTimer: ReturnType<typeof setTimeout> | undefined;
  #watchdogTimer: ReturnType<typeof setInterval> | undefined;
  #reconnectAttempt = 0;
  #lastTrafficAt = 0;

  #channelsByTopic = new Map<string, Channel>();
  #readers = new Map<number, MessageReader>(); // channelId → reader
  /** 期望订阅集合，与服务端实际 advertise 解耦，重连后自动恢复 */
  #handlers = new Map<string, Set<MessageHandler>>();
  #subscriptions = new Map<number, string>(); // subscriptionId → topic
  #subscriptionIdByTopic = new Map<string, number>();

  /** 客户端发布通道：topic → 已 advertise 的 channelId + 编码器 */
  #pubChannels = new Map<string, { channelId: number; writer: MessageWriter }>();

  /**
   * 订阅级节流（毫秒）。0/未设置 = 不节流。
   *
   * 为什么必须放在这里：节流如果只写在 handler 里，消息已经过 readMessage()
   * 反序列化了。像 ExpTraj（76Hz、一次 36 个 marker）或 fsm/path（100Hz）这种
   * 话题，光解码就能把浏览器主线程占满，消息还会堆在 TCP 缓冲里（实测
   * 接收队列涨到 1.1MB），最后表现就是“这条线死活画不出来”。
   * 在解码前直接丢弃，开销降一个量级。
   */
  #throttleByTopic = new Map<string, number>();
  #lastDecodeAt = new Map<string, number>();

  #services = new Map<string, Service>();
  #serviceCallId = 0;
  #pendingCalls = new Map<
    number,
    { service: Service; resolve: (v: unknown) => void; reject: (e: Error) => void }
  >();

  constructor(url: string, opts: FoxgloveConnectionOptions = {}) {
    this.#url = url;
    this.#opts = opts;
    this.#connect();
  }

  get topics(): TopicInfo[] {
    return [...this.#channelsByTopic.values()].map((c) => ({
      topic: c.topic,
      schemaName: c.schemaName,
      schema: c.schema,
    }));
  }

  /**
   * 订阅话题。channel 尚未 advertise 时挂起，advertise 后自动生效。返回退订函数。
   *
   * throttleMs > 0 时对该话题做订阅级节流：距上次解码不足该间隔的消息
   * 会被直接丢弃（不反序列化）。高频可视化话题建议开，比如 200。
   */
  subscribe(topic: string, handler: MessageHandler, throttleMs = 0): () => void {
    let set = this.#handlers.get(topic);
    if (!set) {
      set = new Set();
      this.#handlers.set(topic, set);
    }
    set.add(handler);
    if (throttleMs > 0) this.#throttleByTopic.set(topic, throttleMs);
    this.#ensureSubscribed(topic);

    return () => {
      const s = this.#handlers.get(topic);
      if (!s) return;
      s.delete(handler);
      if (s.size === 0) {
        this.#handlers.delete(topic);
        this.#throttleByTopic.delete(topic);
        this.#lastDecodeAt.delete(topic);
        const subId = this.#subscriptionIdByTopic.get(topic);
        if (subId !== undefined && this.#client) {
          this.#client.unsubscribe(subId);
          this.#subscriptions.delete(subId);
          this.#subscriptionIdByTopic.delete(topic);
        }
      }
    };
  }

  get services(): ServiceInfo[] {
    return [...this.#services.values()].map((s) => ({
      name: s.name,
      type: s.type,
      requestSchema: s.request?.schema ?? s.requestSchema ?? "",
      responseSchema: s.response?.schema ?? s.responseSchema ?? "",
    }));
  }

  /**
   * 客户端发布。schema 必须显式给出（客户端编码需要它）。
   * ROS 1 bridge 要求 encoding 为 "ros1"。
   */
  publish(topic: string, schemaName: string, message: unknown, schema?: string): void {
    if (!this.#client) throw new Error("not connected");
    let ch = this.#pubChannels.get(topic);
    if (!ch) {
      const schemaText = schema ?? findBuiltinSchema(schemaName);
      if (!schemaText) throw new Error(`no schema for ${schemaName}`);
      const channelId = this.#client.advertise({
        topic,
        encoding: ROS1_ENCODING,
        schemaName,
        schema: schemaText,
        // bridge 服务端自己不设这个字段，但 ws-protocol 的类型要它和 schema 成对出现。
        // 用 mcap 规范里 ROS 1 的标准取值。
        schemaEncoding: "ros1msg",
      });
      ch = { channelId, writer: new MessageWriter(parse(schemaText)) };
      this.#pubChannels.set(topic, ch);
    }
    this.#client.sendMessage(ch.channelId, ch.writer.writeMessage(message));
  }

  /** 调用服务。请求/响应 schema 来自服务端 advertise，任何类型都可调。 */
  async callService(name: string, request: unknown): Promise<unknown> {
    const client = this.#client;
    const service = this.#services.get(name);
    if (!client || !service) throw new Error(`service ${name} not available`);
    const reqSchema = service.request?.schema ?? service.requestSchema ?? "";

    let data: Uint8Array;
    if (reqSchema.trim().length === 0) {
      // ROS 1 空请求（如 Trigger）：没有字段就是 0 字节，不存在 CDR 头
      data = new Uint8Array(0);
    } else {
      const writer = new MessageWriter(parse(reqSchema));
      data = writer.writeMessage(request);
    }

    const callId = ++this.#serviceCallId;
    return await new Promise((resolve, reject) => {
      this.#pendingCalls.set(callId, { service, resolve, reject });
      setTimeout(() => {
        if (this.#pendingCalls.delete(callId)) reject(new Error(`service ${name} timeout`));
      }, 10_000);
      client.sendServiceCallRequest({
        serviceId: service.id,
        callId,
        encoding: ROS1_ENCODING,
        data,
      });
    });
  }

  close(): void {
    this.#closed = true;
    if (this.#reconnectTimer) clearTimeout(this.#reconnectTimer);
    if (this.#watchdogTimer) clearInterval(this.#watchdogTimer);
    this.#client?.close();
  }

  #connect(): void {
    this.#opts.onStateChange?.("connecting");
    const protocols = [FoxgloveClient.SUPPORTED_SUBPROTOCOL, "foxglove.sdk.v1"];
    const ws = this.#opts.createWebSocket
      ? this.#opts.createWebSocket(this.#url, protocols)
      : (new WebSocket(this.#url, protocols) as unknown as IWebSocket);

    const client = new FoxgloveClient({ ws });
    this.#client = client;

    client.on("open", () => {
      this.#reconnectAttempt = 0;
      this.#lastTrafficAt = Date.now();
      this.#startWatchdog();
      this.#opts.onStateChange?.("open");
    });

    client.on("serverInfo", () => {
      this.#lastTrafficAt = Date.now();
    });

    client.on("time", () => {
      this.#lastTrafficAt = Date.now();
    });

    client.on("advertise", (channels) => {
      this.#lastTrafficAt = Date.now();
      for (const ch of channels) {
        this.#channelsByTopic.set(ch.topic, ch);
        if (this.#handlers.has(ch.topic)) this.#ensureSubscribed(ch.topic);
      }
      this.#opts.onChannels?.(this.topics);
    });

    client.on("advertiseServices", (services) => {
      for (const s of services) this.#services.set(s.name, s);
      this.#opts.onServices?.(this.services);
    });

    client.on("unadvertiseServices", (serviceIds) => {
      for (const [name, s] of this.#services) {
        if (serviceIds.includes(s.id)) this.#services.delete(name);
      }
      this.#opts.onServices?.(this.services);
    });

    client.on("serviceCallResponse", (res) => {
      const pending = this.#pendingCalls.get(res.callId);
      if (!pending) return;
      this.#pendingCalls.delete(res.callId);
      const schema = pending.service.response?.schema ?? pending.service.responseSchema ?? "";
      try {
        if (schema.trim().length === 0) {
          pending.resolve({});
        } else {
          const reader = new MessageReader(parse(schema));
          pending.resolve(
            reader.readMessage(
              new Uint8Array(res.data.buffer, res.data.byteOffset, res.data.byteLength),
            ),
          );
        }
      } catch (err) {
        pending.reject(new Error(`decode service response failed: ${String(err)}`));
      }
    });

    client.on("serviceCallFailure", (res) => {
      const pending = this.#pendingCalls.get(res.callId);
      if (!pending) return;
      this.#pendingCalls.delete(res.callId);
      pending.reject(new Error(res.message));
    });

    client.on("unadvertise", (channelIds) => {
      for (const [topic, ch] of this.#channelsByTopic) {
        if (channelIds.includes(ch.id)) {
          this.#channelsByTopic.delete(topic);
          this.#readers.delete(ch.id);
          const subId = this.#subscriptionIdByTopic.get(topic);
          if (subId !== undefined) {
            this.#subscriptions.delete(subId);
            this.#subscriptionIdByTopic.delete(topic);
          }
        }
      }
      this.#opts.onChannels?.(this.topics);
    });

    client.on("message", ({ subscriptionId, timestamp, data }) => {
      this.#lastTrafficAt = Date.now();
      const topic = this.#subscriptions.get(subscriptionId);
      if (topic === undefined) return;
      const channel = this.#channelsByTopic.get(topic);
      const handlers = this.#handlers.get(topic);
      if (!channel || !handlers) return;

      // 订阅级节流：必须在 readMessage() 之前返回，否则解码开销照旧。
      const throttle = this.#throttleByTopic.get(topic);
      if (throttle !== undefined && throttle > 0) {
        const tNow = performance.now();
        if (tNow - (this.#lastDecodeAt.get(topic) ?? 0) < throttle) return;
        this.#lastDecodeAt.set(topic, tNow);
      }

      let reader: MessageReader;
      try {
        reader = this.#readerFor(channel);
      } catch (err) {
        this.#opts.onError?.(err instanceof Error ? err : new Error(String(err)));
        return;
      }

      const t0 = performance.now();
      let message: unknown;
      try {
        message = reader.readMessage(data);
      } catch (err) {
        this.#opts.onError?.(
          new Error(`decode failed on ${topic} (${channel.schemaName}): ${String(err)}`),
        );
        return;
      }
      this.#opts.onMessageMeta?.({
        topic,
        bytes: data.byteLength,
        decodeMs: performance.now() - t0,
        serverTimeNs: timestamp,
      });

      // 单个 handler 抛错不影响其他 handler 与后续消息
      for (const h of handlers) {
        try {
          h(message, channel);
        } catch (err) {
          this.#opts.onError?.(
            err instanceof Error ? err : new Error(`handler error on ${topic}: ${String(err)}`),
          );
        }
      }
    });

    client.on("error", (err) => {
      this.#opts.onError?.(err);
    });

    client.on("close", () => {
      if (this.#watchdogTimer) clearInterval(this.#watchdogTimer);
      this.#watchdogTimer = undefined;
      this.#channelsByTopic.clear();
      this.#readers.clear();
      this.#subscriptions.clear();
      this.#subscriptionIdByTopic.clear();
      this.#pubChannels.clear(); // 重连后首次发布时重新 advertise
      this.#services.clear();
      this.#lastDecodeAt.clear();
      for (const [id, p] of this.#pendingCalls) {
        p.reject(new Error("connection closed"));
        this.#pendingCalls.delete(id);
      }
      this.#opts.onStateChange?.("closed");

      const base = this.#opts.reconnectMs ?? 500;
      if (!this.#closed && base > 0) {
        // 指数退避 + 抖动：避免服务重启瞬间被同时重连的客户端打爆
        const backoff = Math.min(base * BACKOFF_FACTOR ** this.#reconnectAttempt, BACKOFF_MAX_MS);
        const jitter = 1 + (Math.random() * 2 - 1) * BACKOFF_JITTER;
        this.#reconnectAttempt++;
        this.#reconnectTimer = setTimeout(() => this.#connect(), backoff * jitter);
      }
    });
  }

  /**
   * 半开连接看门狗：TCP 断链（Wi-Fi 掉线、网线拔出）时本端可能长时间
   * 收不到 close 事件。有订阅却长期无任何服务端消息时主动断开触发重连。
   */
  #startWatchdog(): void {
    const timeout = this.#opts.staleTimeoutMs ?? 5000;
    if (timeout <= 0) return;
    if (this.#watchdogTimer) clearInterval(this.#watchdogTimer);
    this.#watchdogTimer = setInterval(() => {
      if (this.#subscriptionIdByTopic.size === 0) {
        this.#lastTrafficAt = Date.now(); // 无订阅时无预期流量，不判卡死
        return;
      }
      if (Date.now() - this.#lastTrafficAt > timeout) {
        this.#opts.onError?.(new Error(`connection stale (>${timeout}ms silent), reconnecting`));
        this.#client?.close();
      }
    }, 1000);
  }

  #ensureSubscribed(topic: string): void {
    if (!this.#client) return;
    if (this.#subscriptionIdByTopic.has(topic)) return;
    const channel = this.#channelsByTopic.get(topic);
    if (!channel) return; // 等 advertise
    const subId = this.#client.subscribe(channel.id);
    this.#subscriptions.set(subId, topic);
    this.#subscriptionIdByTopic.set(topic, subId);
  }

  #readerFor(channel: Channel): MessageReader {
    let reader = this.#readers.get(channel.id);
    if (!reader) {
      if (channel.encoding !== ROS1_ENCODING) {
        throw new Error(
          `unsupported encoding "${channel.encoding}" on ${channel.topic} (expected "${ROS1_ENCODING}")`,
        );
      }
      if (!channel.schema || channel.schema.trim().length === 0) {
        throw new Error(`empty schema for ${channel.topic} (${channel.schemaName})`);
      }
      reader = new MessageReader(parse(channel.schema));
      this.#readers.set(channel.id, reader);
    }
    return reader;
  }
}

/**
 * 客户端发布需要的 schema 文本。ROS 1 用 .msg 文本格式。
 * 这里只放地面站真正会发的那几个最小集合；其余一律用服务端 advertise 的 schema。
 */
const BUILTIN_SCHEMAS: Record<string, string> = {
  "geometry_msgs/PoseStamped": [
    "std_msgs/Header header",
    "geometry_msgs/Pose pose",
    "===",
    "MSG: std_msgs/Header",
    "uint32 seq",
    "time stamp",
    "string frame_id",
    "===",
    "MSG: geometry_msgs/Pose",
    "geometry_msgs/Point position",
    "geometry_msgs/Quaternion orientation",
    "===",
    "MSG: geometry_msgs/Point",
    "float64 x",
    "float64 y",
    "float64 z",
    "===",
    "MSG: geometry_msgs/Quaternion",
    "float64 x",
    "float64 y",
    "float64 z",
    "float64 w",
  ].join("\n"),

  "geometry_msgs/PoseWithCovarianceStamped": [
    "std_msgs/Header header",
    "geometry_msgs/PoseWithCovariance pose",
    "===",
    "MSG: geometry_msgs/PoseWithCovariance",
    "geometry_msgs/Pose pose",
    "float64[36] covariance",
    "===",
    "MSG: geometry_msgs/Pose",
    "geometry_msgs/Point position",
    "geometry_msgs/Quaternion orientation",
    "===",
    "MSG: geometry_msgs/Point",
    "float64 x",
    "float64 y",
    "float64 z",
    "===",
    "MSG: geometry_msgs/Quaternion",
    "float64 x",
    "float64 y",
    "float64 z",
    "float64 w",
    "===",
    "MSG: std_msgs/Header",
    "uint32 seq",
    "time stamp",
    "string frame_id",
  ].join("\n"),
};

function findBuiltinSchema(schemaName: string): string | undefined {
  return BUILTIN_SCHEMAS[schemaName];
}
