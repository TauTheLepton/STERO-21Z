# Sprawozdanie STERO blok manipulacyjny

## Projekt 1

### Opis działania programu

Nasz programstopniowo ewoluował. Zaczęliśmy od najprostszej wersji, czyli takiej w której wszystkie pozycje są na stałe zapisane w kodzie w konfiguracji złączy.
Następnie dodaliśmy planner który sam odnajdował optymalną trajektorię rucho od pozycji początkowej do słoja i następnie do pozycji przed odstawieniem.
Już to działanie pozwoliło nam z wielu pozycji pośrednich zapisanych w kodie zredukować ich liczbę do zaledwie 3 (wliczając pozycję początkową do której program sam wracał zawsze po odstawieniu słoja).
Kolejnym krokiem było dodanie sterowania w trybie zadawania pozycji końcówki manipulatora jednej ręki w przestrzeni kartezjańskiej związanej z bazowym układem współrzędnych.
Do tego potrzeba było odczytać pozycję słoja ze świata rzeczywistego, co zrobiliśmy wczytując pozycję słoja prosto z programu gazebo symulującgo naszą rzeczywistość.
Do pozycji słoja dodaliśmy stały offset tak, żeby manipulator zatrzymał się kawałem przed słoikiem, żeby mógł go normalnie chwycić.
Do tej pory wszystko działało poprawnie i rozwiązanie opisane do tego momntu znajduje się w pliku `pickup_laydown.py`.
