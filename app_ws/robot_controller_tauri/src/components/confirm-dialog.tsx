import { useConfirmStore } from "@/stores/confirm";

export function ConfirmDialog() {
  const message = useConfirmStore((s) => s.message);
  const answer = useConfirmStore((s) => s.answer);
  if (message == null) return null;
  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center bg-black/60 p-4">
      <div className="card mb-0 w-full max-w-md border-line">
        <p className="whitespace-pre-wrap text-sm">{message}</p>
        <div className="row mt-4 justify-end">
          <button className="btn-secondary" onClick={() => answer(false)}>
            キャンセル
          </button>
          <button className="btn" autoFocus onClick={() => answer(true)}>
            実行
          </button>
        </div>
      </div>
    </div>
  );
}
