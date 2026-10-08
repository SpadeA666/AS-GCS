/**
 * 数字输入框（受控）。
 *
 * 为什么不用 `<input type="number" value={n} onChange={e => setN(parseFloat(e.target.value) || 0)}>`：
 *   用户按退格清空时 value 是 ""，parseFloat("") 得到 NaN，`NaN || 0` 兜底成 0，
 *   state 立刻回填字符串 "0" —— 于是那个 0 永远删不掉，每次输入前都得先删它。
 *
 * 这里的做法：把「用户正在敲的原始文本」单独存成 state，允许中间态为空字符串，
 * 只在失焦（或回车）时才解析、夹到 min/max 并回写。外部值变化时若不在编辑中也会同步。
 */
import { useEffect, useState } from "react";

export interface NumberFieldProps {
  value: number;
  onValueChange: (v: number) => void;
  step?: string | number;
  min?: number;
  max?: number;
  title?: string;
  disabled?: boolean;
  className?: string;
}

export function NumberField({
  value,
  onValueChange,
  step = "0.1",
  min,
  max,
  title,
  disabled,
  className = "num",
}: NumberFieldProps) {
  const [text, setText] = useState(() => String(value));
  const [editing, setEditing] = useState(false);

  // 外部改值（比如程序重置、切换航点）时同步显示；正在输入则不打断
  useEffect(() => {
    if (!editing) setText(String(value));
  }, [value, editing]);

  const commit = () => {
    setEditing(false);
    const n = parseFloat(text);
    if (!Number.isFinite(n)) {
      setText(String(value)); // 非法输入 → 还原
      return;
    }
    let v = n;
    if (min !== undefined) v = Math.max(min, v);
    if (max !== undefined) v = Math.min(max, v);
    onValueChange(v);
    setText(String(v));
  };

  return (
    <input
      className={className}
      type="number"
      inputMode="decimal"
      step={step}
      min={min}
      max={max}
      title={title}
      disabled={disabled}
      value={text}
      onFocus={() => setEditing(true)}
      onChange={(e) => setText(e.target.value)} // 原样存，允许空
      onBlur={commit}
      onKeyDown={(e) => {
        if (e.key === "Enter") (e.target as HTMLInputElement).blur();
      }}
    />
  );
}
