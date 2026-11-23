import { create } from "zustand";

// WKWebView(Tauri/wry)では window.confirm が未実装で常に false を返すため、
// アプリ内モーダルで確認を取る。ConfirmDialog（app-shell 常駐）が message を描画し、
// ユーザーの回答を answer() で保留中の Promise へ返す。
interface ConfirmStore {
  message: string | null;
  resolve: ((ok: boolean) => void) | null;
  ask: (message: string) => Promise<boolean>;
  answer: (ok: boolean) => void;
}

export const useConfirmStore = create<ConfirmStore>((set, get) => ({
  message: null,
  resolve: null,
  ask: (message) =>
    new Promise<boolean>((resolve) => {
      // 表示中の確認があればキャンセル扱いで閉じてから新しい方を出す。
      get().resolve?.(false);
      set({ message, resolve });
    }),
  answer: (ok) => {
    get().resolve?.(ok);
    set({ message: null, resolve: null });
  },
}));

/** window.confirm の代替（非同期）。呼出し側は `await confirmAsync(msg)` とする。 */
export const confirmAsync = (message: string) =>
  useConfirmStore.getState().ask(message);
