# ROSとOpenCVを使ったマウスカーソルの座標取得プログラム

[![](https://img.youtube.com/vi/CQ2jJM5RRYg/0.jpg)](https://www.youtube.com/watch?v=CQ2jJM5RRYg)

- ** mouse_cursor.cpp**では静止画像が表示されてるウインドウのカーソル座標を取得してます。

  - /home/hirota/catkin_ws/src/for_blog/mouse_curcor/lena.jpg"の部分は各自の画像の絶対パスを入れてください


- ** mouse_corsor2 ** では動画像が表示されてるウインドウのカーソル座標を取得してます。

   - /usb_cam/image_rawの名前で動画をパブリッシュしてください。

     筆者の環境では、
> $ roslaunch usb_cam usb_cam-test.launch

      を実行しています。
