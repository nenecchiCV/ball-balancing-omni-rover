---
name: using-simulink-agentic-toolkit
description: Simulink モデルの理解、作成、編集、シミュレーション、検証に Simulink Agentic Toolkit の MCP ツールとドメインスキルを利用する。
---

# Simulink Agentic Toolkit の利用

Simulink に関する作業では、[Simulink Agentic Toolkit](https://github.com/matlab/simulink-agentic-toolkit) が提供する MCP ツールと、作業内容に対応するドメインスキルを組み合わせる。

## セットアップ

ユーザーが toolkit の導入を求めた場合は、toolkit を取得したフォルダーを指定して、次のコマンドを MATLAB で実行するよう案内する。エージェントはこのセットアップコマンドをユーザーに代わって実行しない。

```matlab
addpath('<path-to-setup-folder>')
setupAgenticToolkit("install")
```

導入済みの環境を更新する場合は `setupAgenticToolkit("update")`、状態を確認する場合は `setupAgenticToolkit("status")` を案内する。

## 作業時の選択

1. タスクに対応する toolkit のドメインスキルを読み、その手順と制約を適用する。
2. モデルの把握には `model_overview`、構造や式の確認には `model_read` を優先する。
3. パラメーター値の確認では、必要に応じて `model_query_params` と `model_resolve_params` を使い分ける。
4. モデルの構造変更には `model_edit` を使用し、変更後は `model_check` で構造を確認する。
5. 振る舞いの検証が必要な場合は、利用可能なテスト資産を確認して `model_test` または適切なシミュレーション手段を使用する。

## 接続の確認

MCP ツールを利用できない場合は、MATLAB が起動していることと、その MATLAB セッションで toolkit が初期化されていることを確認する。必要に応じて、toolkit のインストール先を指定して次を MATLAB で実行するよう案内する。

```matlab
addpath('<toolkit-root>')
satk_initialize
```

利用できない製品やライセンスを前提にせず、現在の環境で可能な確認方法を選ぶ。重要なモデル変更とツール呼び出しの内容は、実行前後に対象と結果を確認する。
