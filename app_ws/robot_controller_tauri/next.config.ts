import type { NextConfig } from "next";

const nextConfig: NextConfig = {
  output: "export",
  // Tauri の WebView から相対パスで配信するため、assetPrefix は既定のまま
  // `out/` へ静的エクスポートする。
};

export default nextConfig;
