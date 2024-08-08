# map saveする方法

```
cd ~/sick_wagen_ws/src/sick_wagen/shellscript
./map-saver.sh
```

上記スクリプトでsick_wagen_ws/mapsフォルダに保存されます。
amclなどでこのmapファイルを使用したい場合、下記スクリプトで環境変数ROS\_MAPを設定してください。

```
export ROS_MAP=/home/sick/sick_wagen_ws/maps/map_2023-05-03-135859
```

.bashrcで下記のように設定すると、bash起動時に環境変数ROS\_MAPが初期化されます。

```
echo "export ROS_MAP=~/sick_wagen_ws/maps/map_2023-05-03-132355" >> .bashrc
source ~/.bashrc
```