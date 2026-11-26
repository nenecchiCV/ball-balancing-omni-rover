// 俯瞰オドメトリマップ。ワールド座標（x 右、y 上、原点=ロボット起動点）を描画する。

import { useEffect, useRef } from "react";

import type { PoseTelemetry } from "@/lib/types";

interface Props {
  pose: PoseTelemetry | null;
  trail: [number, number][];
  target?: [number, number] | null;
  scale?: number; // px / m
}

export function OdometryMap({ pose, trail, target, scale = 80 }: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    const w = canvas.width;
    const h = canvas.height;
    const cx = w / 2;
    const cy = h / 2;
    const toX = (x: number) => cx + x * scale;
    const toY = (y: number) => cy - y * scale;

    ctx.fillStyle = "#101418";
    ctx.fillRect(0, 0, w, h);

    // 1 m グリッド
    ctx.strokeStyle = "#232a33";
    ctx.lineWidth = 1;
    const step = scale;
    for (let gx = cx % step; gx < w; gx += step) {
      ctx.beginPath();
      ctx.moveTo(gx, 0);
      ctx.lineTo(gx, h);
      ctx.stroke();
    }
    for (let gy = cy % step; gy < h; gy += step) {
      ctx.beginPath();
      ctx.moveTo(0, gy);
      ctx.lineTo(w, gy);
      ctx.stroke();
    }

    // 原点軸
    ctx.strokeStyle = "#3a4552";
    ctx.beginPath();
    ctx.moveTo(0, cy);
    ctx.lineTo(w, cy);
    ctx.moveTo(cx, 0);
    ctx.lineTo(cx, h);
    ctx.stroke();

    // 軌跡
    if (trail.length > 1) {
      ctx.strokeStyle = "#4ea1ff";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(toX(trail[0][0]), toY(trail[0][1]));
      for (const [x, y] of trail) ctx.lineTo(toX(x), toY(y));
      ctx.stroke();
    }

    // 目標点
    if (target) {
      ctx.strokeStyle = "#ffb454";
      ctx.lineWidth = 2;
      const tx = toX(target[0]);
      const ty = toY(target[1]);
      ctx.beginPath();
      ctx.moveTo(tx - 8, ty);
      ctx.lineTo(tx + 8, ty);
      ctx.moveTo(tx, ty - 8);
      ctx.lineTo(tx, ty + 8);
      ctx.stroke();
      ctx.beginPath();
      ctx.arc(tx, ty, 5, 0, Math.PI * 2);
      ctx.stroke();
    }

    // ロボット（向きつき三角）
    if (pose && pose.x != null && pose.y != null && pose.yaw != null) {
      const px = toX(pose.x);
      const py = toY(pose.y);
      const r = 10;
      ctx.fillStyle = "#5be07a";
      ctx.beginPath();
      ctx.moveTo(px + r * Math.cos(pose.yaw), py - r * Math.sin(pose.yaw));
      ctx.lineTo(
        px + r * Math.cos(pose.yaw + 2.5),
        py - r * Math.sin(pose.yaw + 2.5),
      );
      ctx.lineTo(
        px + r * Math.cos(pose.yaw - 2.5),
        py - r * Math.sin(pose.yaw - 2.5),
      );
      ctx.closePath();
      ctx.fill();
    }

    // スケール表示
    ctx.fillStyle = "#8b98a8";
    ctx.font = "12px monospace";
    ctx.fillText("1m", cx + step + 4, cy + 14);
  }, [pose, trail, target, scale]);

  return (
    <canvas
      ref={canvasRef}
      width={560}
      height={420}
      className="max-w-full rounded-lg border border-line bg-[#101418]"
    />
  );
}
