#### cmd_vel_tsukuba2022.cpp
つくばチャレンジ2022の実験走行・本走行で使用した。
cmd_velの出力を通常走行・一時停止・信号認識の座標ごと変更する。
* 通常走行
    movebaseのcmd_velを出力
* 一時停止
    JoyStickのボタンを押すとGo
* 信号認識
    青信号が５フレーム連続で検出されるとGo

waipointのtarget_poseが指定の位置になるとnavigation_status(0: 通常, 1: 一時停止, 2: 信号認識)を変更する。
