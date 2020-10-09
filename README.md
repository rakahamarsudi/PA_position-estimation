# PA_position-estimation
Repository ini berisi program proyek akhir mengenai estimasi posisi soccer robot menggunakan kamera omnidirectional.
Program ini akan mengestimasi posisi berdasarkan perhitungan jarak robot dari tiang gawang melalui citra omnidirectional.

Penulisan program dilakukan mengunakan C++ dengan bantuan OpenCV dan ROS

Video bahan uji coba : oldEdit.mp4
> https://drive.google.com/file/d/19gdENWA6crEgiXEW2gX_lPfN7uF4vOro/view?usp=sharing

Desain ilustrasi lapangan : desain_Lapangan_new.png
> https://drive.google.com/file/d/1VLcKeOS9qyuZXceuPD9CaCnYccwt6tFg/view?usp=sharing

build project :
>catkin_make

set up ROS :
>source devel/setup.bash

run roscore :
>roscore

   [RUN PROJECT]
(using publisher & subscriber)
publisher using video :
>rosrun try_camera vid_publisher /home/user/oldEdit.mp4 
subscriber :
>rosrun try_opencv try_all 
   
   or
   
run with ilustration in one program:
>rosrun try_opencv try_opencv /home/rakasiwi/oldEdit.mp4 /home/user/desain_Lapangan_new.png 


