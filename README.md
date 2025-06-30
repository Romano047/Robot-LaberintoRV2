# Robot-LaberintoRV2
Robot Laberinto R 2024-2025

Proyecto realizado en la Escuela de Educación Secundaria Técnica N°5 "2 de Abril" Temperley, Buenos Aires, Argentina. Desarrollado por Iván Romano, estudiante de Electrónica de la institución a cargo del profesor Ing. Martín Leguizamón.

El objetivo de este reoositrio es el de desarrollar un robot autónomo de competición dentro de la categoría de "Robot Laberinto". 

Se tomarán como referencia en su elaboración las reglas actualmente vigentes (2024) dentro de la Liga Nacional de Robótica (LNR) en la categoría de "Laberinto". 


# Guía de Repositorio

- Hardware
  - KiCad: En ella se encuentran todos los archivos correspondientes al desarrollo del PCB.

- 3D
  - AutoCad: En esta carpeta se encuentran los archivos correspondientes al diseño 3D en conjunto a sus trazados a formato .stl para facilitar su posterior impresión. 

- Software
  - Ensayos: Ensayos básicos de cada funcionalidad o punto de interés de forma aislada que conforman el robot. (A su vez se encontrarán en el archivos correspondientes al robort de competición provincial "Alita").
  - LaberintoR2_v2: Versión del Software ejecutada actualmente en el robot. 

# Especificaciones

- Software: Desarrollado en C Orientado a Objetos, por medio del uso de VsCode en conjunto a su extensión PlatformIO.
- Desarrollo de PCBs: KiCad 7
- Placa de Desarrollo: Espressif ESP-32 WROOM DEV KIT
- Sensores de proximidad: HC-SR04
- Drivers de Motores: TB6612fng
