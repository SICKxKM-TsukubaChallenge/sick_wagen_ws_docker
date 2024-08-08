## GPSのポート

環境変数からシリアル通信のポートを撮ってきています。  
ubloxのポートを調べ、下記のように設定してから、launchしてください。  
環境によっては、/dev/ttyACM1などになる可能性があります。  

```
echo "export TTY_UBLOX=/dev/ttyACM0" >> .bashrc
```