import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

export default defineConfig({
  plugins: [react()],
  server: {
    port: 5173,
    // 允许局域网访问，方便日后从别的机器打开
    host: true,
  },
});
