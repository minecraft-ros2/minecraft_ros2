# RViz2
Rvizとは、ROS（Robot Operating System）に付属する3Dビジュアライザーです。

Rvizを使うことで、プレーヤー情報やLiDAR、カメラなどのデータを3D空間上で表示することができます。

## 起動
```bash
rviz2
```
たったこれだけです(sourceは実行済みの前提です)
![デフォルト画面](/images/tutorial/rviz2_default.png)

## 設定
ここではImageとLiDARの可視化を行いたいと思います
(minecraft_ros2を起動しておいてください)

### Image
Imageは画像で、マイクラのプレイ画面が送信されています。

画面左下AddからBy topicでImageを選択しましょう
![選択画面](/images/tutorial/rviz2_add_image.png)

そうするとマインクラフトの画面が左下に表示されます！

![表示された](/images/tutorial/rviz2_image.png)

### LiDAR
[LiDARセンサーを装備](/jp/documentation/doc_sensors)することで周囲のブロックやエンティティの距離がわかるようになります。

Imageと同様にBy topicで Point Cloud 2を選択しましょう

表示されるかと思いきや何も変化がありません。

なぜなら表示する座標系を間違えているからです。

Global OptionsのFixed Frameを`player`に変更することで表示されるようになります。

![変更後](/images/tutorial/rviz2_fixed_frame.png)

もし見づらかったらPoint Cloud 2 の Size (m) を変更することで見やすくすることができます。

