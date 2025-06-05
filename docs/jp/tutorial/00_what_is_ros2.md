# ROS 2とは?
ROSとは、Robot Operating Systemの略で、ロボット開発のために必要な機能が備わっています
- plumbing（通信）
- tools（ツール群）
- capabilities（機能群）
- ecosystem（エコシステム）
が含まれます。

## plumbing（通信）
- ROSノード(=個々のプログラム)の疎な結合
    - 再配置が簡単
- 基本となる型を組み合わせ新しい型を定義できる
    - bool, int, doubleはもちろん配列も
    - 標準搭載されているメッセージで時間情報を付け加えたりできる
- DDSを使用しプログラム間の通信相手は自動で解決される

### topic（トピック）
- 非同期型
- pub/sub通信（多対多）

### service（サービス）
- 同期型
- 1対1

### action（アクション）
- フィードバック付きサービス
- フィードバックは非同期

### parameter（パラメーター）
- 多変量辞書

## tools（ツール群）
### Colcon
- ビルドシステム
- C++,C,Python,Javaなどあらゆる言語を一括でビルドできる

### launch
- 複数のプログラムを同時起動
- パラメーターの指定なども可能

### rqt, rviz
- 情報可視化ツール

## capabilities（機能群）
- 2000以上のライブラリ
- 使いたいセンサーのライブラリを探すだけですぐに使える

### TF
- 3次元の姿勢と連鎖関係
- 時間管理

### ros2_control
- ハードウェア抽象化
- 色々な種類の制御を試せる

### Navigation 2
- 地図ベースの位置推定や経路計画、追従、障害物回避
- 2Dの自動運転

### MoveIt
- マニピュレーションの制御

## ecosystem（エコシステム）
- 便利なツール、豊富なライブラリなどはすべてオープンソース
- メンテナンスや共有の仕組みが整っている
- ROSConやROS Discourseなどで交流
- ROS Japan Users Group (rosjp) で自由に発表＆交流
- ROSConJPはROSConの日本版