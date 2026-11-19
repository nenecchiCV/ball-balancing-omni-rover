"use client";

// バーチャルジョイスティック。中心からの正規化オフセット (-1..1) を onChange で返す。
// y は上方向が正。解放時は (0, 0) を返す。

import { useCallback, useRef, useState } from "react";

interface Props {
  label: string;
  axes?: "xy" | "x"; // xy: 2軸、x: 横のみ（回転用）
  onChange: (x: number, y: number) => void;
}

export function Joystick({ label, axes = "xy", onChange }: Props) {
  const padRef = useRef<HTMLDivElement>(null);
  const [pos, setPos] = useState({ x: 0, y: 0 });

  const update = useCallback(
    (clientX: number, clientY: number) => {
      const pad = padRef.current;
      if (!pad) return;
      const rect = pad.getBoundingClientRect();
      const r = rect.width / 2;
      let dx = (clientX - (rect.left + r)) / r;
      let dy = -((clientY - (rect.top + r)) / r);
      const len = Math.hypot(dx, dy);
      if (len > 1) {
        dx /= len;
        dy /= len;
      }
      if (axes === "x") dy = 0;
      setPos({ x: dx, y: dy });
      onChange(dx, dy);
    },
    [axes, onChange],
  );

  const release = useCallback(() => {
    setPos({ x: 0, y: 0 });
    onChange(0, 0);
  }, [onChange]);

  return (
    <div className="mx-0 mb-3 mr-6 mt-3 inline-block text-center">
      <div
        ref={padRef}
        className="relative size-40 cursor-grab touch-none rounded-full border border-line bg-[#141a22] active:cursor-grabbing"
        onPointerDown={(e) => {
          e.currentTarget.setPointerCapture(e.pointerId);
          update(e.clientX, e.clientY);
        }}
        onPointerMove={(e) => {
          if (e.buttons > 0) update(e.clientX, e.clientY);
        }}
        onPointerUp={release}
        onPointerCancel={release}
      >
        <div
          className="pointer-events-none absolute left-1/2 top-1/2 size-14 rounded-full bg-[#2456a6]"
          style={{
            transform: `translate(calc(-50% + ${pos.x * 40}px), calc(-50% + ${-pos.y * 40}px))`,
          }}
        />
      </div>
      <div className="mt-2 font-mono text-[13px] text-muted">
        {label} ({pos.x.toFixed(2)}, {pos.y.toFixed(2)})
      </div>
    </div>
  );
}
