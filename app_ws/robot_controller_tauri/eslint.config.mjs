import js from "@eslint/js";
import reactHooks from "eslint-plugin-react-hooks";
import reactRefresh from "eslint-plugin-react-refresh";
import globals from "globals";
import tseslint from "typescript-eslint";

export default tseslint.config([
  {
    ignores: [
      "dist/**",
      "node_modules/**",
      "src-tauri/**",
      "src/lib/bindings.ts",
      "vite.config.ts",
    ],
  },
  {
    files: ["src/**/*.{ts,tsx}"],
    extends: [
      js.configs.recommended,
      ...tseslint.configs.recommended,
      reactHooks.configs.flat.recommended,
      reactRefresh.configs.vite,
    ],
    languageOptions: {
      ecmaVersion: 2022,
      globals: globals.browser,
    },
    rules: {
      // IPC/store との同期で effect 内 setState を用いる既存パターン
      // （接続時リロード・切断時リセット・props→入力同期）と衝突するため無効化。
      "react-hooks/set-state-in-effect": "off",
    },
  },
]);
