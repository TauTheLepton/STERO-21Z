# STERO - Sprawozdanie blok manipulacyjny

## Projekt 1

### Cel zadania

Celem projektu było napisanie programu sterującego robotem Velma, który spowoduje, że robot chwyci, przeniesie i odłoży obiekt w inne miejsce. Zadanie to może być zaprogramowanie z różnym stopniem elsatyczności algorytmu. W ogólności robot powinien dobrze zareagować na wydane podniesienie i odłożenie przedmiotu w dowolnej konfigracji środowiska. Na etapie oddania projektu robot potrafi zdecydować, czy zadanie ma sens (jest w stanie po niego sięgnąć), sięgnąć po obiekt, jeżeli znajduje się w zasięgu i odstawić go na drugi stolik w jedno określone miejsce).

### Pliki i Implementacja

Implemenatcja zadania znajduje się w pliku `pickup_laydown.py` (wersja dokończona, ale nie uwzględniająca odwrotnej kinematyki robota) oraz `my_pickup_laydown.py` (wersja niedokończona z próba uwzlędnienia i przeliczania odwrotnej kinematyki robota). Paczka zawiera skrypt do budowy pełnej octomapy `create_octomap.py`, octomapa używana przy testowaniu zadania to `new_octmap.bt`.


### Rozwój podejścia, zmiany koncepcji i implementacji

Program stopniowo ewoluował. Zaczęto od najprostszej wersji, czyli takiej w której wszystkie pozycje są na stałe zapisane w kodzie w konfiguracji złączy. Stworzono octomapę serwerem online, zapisano ją do pliku i korzytano z jej poprzez serwer offline.<br>
Następnie dodano planner który sam odnajdował optymalną trajektorię rucho od pozycji początkowej do szklanki i następnie do pozycji przed odstawieniem.
To działanie pozwoliło nam z wielu pozycji pośrednich zapisanych w kodie zredukować ich liczbę do zaledwie 3 (wliczając pozycję początkową do której program sam wracał zawsze po odstawieniu słoja).<br>
Kolejnym krokiem było dodanie sterowania w trybie zadawania pozycji końcówki manipulatora jednej ręki w przestrzeni kartezjańskiej związanej z bazowym układem współrzędnych.
Do tego potrzeba było odczytać pozycję szklanki ze świata rzeczywistego, co zrobiliśmy wczytując pozycję szklanki prosto z programu gazebo symulującgo naszą rzeczywistość.
Do pozycji szklanki dodaliśmy stały offset tak, żeby manipulator zatrzymał się kawałem przed słoikiem, żeby mógł go normalnie chwycić. To podejście pozwala na chwycenie szkalnki, która znajduje się w dowolym miejsu osiągalnym przez ramię robota Velma.
Do tej pory wszystko działało poprawnie i rozwiązanie opisane do tego momentu znajduje się w pliku `pickup_laydown.py`.

### Przestrzeń operacyjna robota

Została określona przy pomocy skryptu `reachability_range.py` z paczki `velma_common`. Została dopasowana do środowiska eksperymentalnie. Przestrzeń fioletowa to obszar, w którego zasięgu robot może dokonywać operacji.

![alt text](https://github.com/STERO-21Z/zubik-palczuk/blob/tiago/manipulation/data/rys/sphere_x_y.png)

![alt text](https://github.com/STERO-21Z/zubik-palczuk/blob/tiago/manipulation/data/rys/sphere_z.png)

### Struktura programu

Węzeł `pickup_laydown` został zaimplementowany przy wykorzystaniu automatu skończonego. Jego schemat został zaprezentowany poniżej.<br>
![alt text](https://github.com/STERO-21Z/zubik-palczuk/blob/tiago/manipulation/data/rys/automat.png)

### Działanie programu (środowisko Gazebo)

Inicjalizacja robota.

![alt text](https://github.com/STERO-21Z/zubik-palczuk/blob/tiago/manipulation/data/rys/robot_state1.png)

Ruch do pozycji pośredniej (jimp).

![alt text](https://github.com/STERO-21Z/zubik-palczuk/blob/tiago/manipulation/data/rys/robot_state2.png)

Ruch końcówki w kieunku szklanki (cimp).

![alt text](https://github.com/STERO-21Z/zubik-palczuk/blob/tiago/manipulation/data/rys/robot_state3.png)

Zamknięcie chwytaków, złapanie szklanki.

![alt text](https://github.com/STERO-21Z/zubik-palczuk/blob/tiago/manipulation/data/rys/robot_state4.png)

Uniesienie szklanki (cimp).

![alt text](https://github.com/STERO-21Z/zubik-palczuk/blob/tiago/manipulation/data/rys/robot_state5.png)

Przełożenie szklanki na drugi stolik (jimp).

![alt text](https://github.com/STERO-21Z/zubik-palczuk/blob/tiago/manipulation/data/rys/robot_state6.png)

Otworzenie chwytaka.

![alt text](https://github.com/STERO-21Z/zubik-palczuk/blob/tiago/manipulation/data/rys/robot_state7.png)

Uniesienie chwytaka nad szklankę (cimp).

![alt text](https://github.com/STERO-21Z/zubik-palczuk/blob/tiago/manipulation/data/rys/robot_state8.png)

Ruch do pozycji początkowej (jimp).

![alt text](https://github.com/STERO-21Z/zubik-palczuk/blob/tiago/manipulation/data/rys/robot_state9.png)

### Uruchomienie skryptu

Reset buffers

>rosrun velma_common reset_shm_comm.py

Manipulation velma_system.launch

>roslaunch  manipulation velma_system.launch

Planner launch

>roslaunch velma_ros_plugin velma_planner.launch

RViz

>rosrun rcprg_ros_utils run_rviz.sh

Gazebo

>roslaunch rcprg_gazebo_utils gazebo_client.launch

Octomap offline

>roslaunch velma_common octomap_offline_server.launch  octomap_file:=/home/student/mobile_ws/src/zubik-palczuk/manipulation/data/new_octmap.bt

Table_a gazebo position

>roslaunch rcprg_gazebo_utils gazebo_publish_ros_tf_object.launch link_name:=table_a::link frame_id:=table_a

Table_b gazebo position

>roslaunch rcprg_gazebo_utils gazebo_publish_ros_tf_object.launch link_name:=table_b::link frame_id:=table_b

Jar hollow gazebo position

>roslaunch rcprg_gazebo_utils gazebo_publish_ros_tf_object.launch link_name:=jar_hollow::link frame_id:=jar

Bowl high gazebo position

>roslaunch rcprg_gazebo_utils gazebo_publish_ros_tf_object.launch link_name:=bowl_high::link frame_id:=bowl

Home motors

>rosrun velma_task_cs_ros_interface initialize_robot.py

Pickup_laydown script

>cd /.../manipulation/scripts 
>python pickup_laydown.py

