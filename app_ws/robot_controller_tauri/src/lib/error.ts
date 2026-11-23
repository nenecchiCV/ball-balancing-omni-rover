import type { AppError } from "@/lib/bindings";

const KIND_LABEL: Record<AppError["kind"], string> = {
  ble: "BLE エラー",
  notConnected: "未接続",
  timeout: "タイムアウト",
  badResponse: "不正な応答",
  invalidParam: "パラメータ不正",
  rejected: "コマンド拒否",
  send: "送信失敗",
};

/**
 * IPC の AppError（`{ kind, message? }` タグ付きオブジェクト）や Error を
 * 表示用文字列へ変換する。String(e) だと [object Object] になるため必ずこれを使う。
 */
export function formatError(e: unknown): string {
  if (e != null && typeof e === "object" && "kind" in e) {
    const err = e as AppError;
    const label = KIND_LABEL[err.kind] ?? err.kind;
    return "message" in err && typeof err.message === "string"
      ? `${label}: ${err.message}`
      : label;
  }
  if (e instanceof Error) return e.message;
  return String(e);
}
