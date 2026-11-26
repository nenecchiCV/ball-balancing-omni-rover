// ルート定義: パス → feature ページの対応のみを記述する薄い層。
// 画面実体は features/<domain>/、共通シェルは components/app-shell。

import { Route, Routes } from "react-router";

import { AppShell } from "@/components/app-shell";
import { ConnectionPage } from "@/features/connection/connection-page";
import { HomePage } from "@/features/home/home-page";
import { LogsPage } from "@/features/logs/logs-page";
import { OdometryPage } from "@/features/odometry/odometry-page";
import { PositionPage } from "@/features/position/position-page";
import { RcPage } from "@/features/rc/rc-page";
import { TestPage } from "@/features/test/test-page";
import { TuningPage } from "@/features/tuning/tuning-page";

export function App() {
  return (
    <AppShell>
      <Routes>
        <Route path="/" element={<HomePage />} />
        <Route path="/connect" element={<ConnectionPage />} />
        <Route path="/rc" element={<RcPage />} />
        <Route path="/position" element={<PositionPage />} />
        <Route path="/odometry" element={<OdometryPage />} />
        <Route path="/logs" element={<LogsPage />} />
        <Route path="/test" element={<TestPage />} />
        <Route path="/tuning" element={<TuningPage />} />
      </Routes>
    </AppShell>
  );
}
