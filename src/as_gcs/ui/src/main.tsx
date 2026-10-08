import React from "react";
import { createRoot } from "react-dom/client";
import App from "./app/App";
import "./app/app.css";

const el = document.getElementById("root");
if (!el) throw new Error("#root not found");
createRoot(el).render(
  <React.StrictMode>
    <App />
  </React.StrictMode>,
);
