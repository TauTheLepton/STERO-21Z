# Komendy
## sterowanie z klawiatury
rosrun teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=key_vel
## zapisywanie mapy
rosrun map_server map_saver -f moja_mapa
## dodawanie do startu robota (chyba mapy)
<node name="map_server" pkg="map_server" type="map_server" args="$(find stero_mobile_init)/maps/moja_mapa.yaml"/>