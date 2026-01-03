# 🔧 Module MGD (Modèle Géométrique Direct)

Le module `mgd` implémente le modèle géométrique direct du robot à l’aide de la convention de Denavit–Hartenberg modifié.
Il permet de calculer :

la position de l’organe terminal,
son orientation,
la matrice homogène complète.


## Code prêt à l’utilisation (à copier-coller dans `main.py`)

`import numpy as np`

`import configuration as cf`

`from mgd.mgd import Mgd`

### Angles articulaires (en radians)
`q = [0, np.pi/6, -np.pi/4, np.pi/3, np.pi/6, 0]`

### Initialisation du MGD
`robot = Mgd(cf.data, q)`

### Position de l’effecteur final
`position = robot.get_position()`

`print("Position :", position)`

### Orientation de l’effecteur final
`orientation = robot.get_oriontation()`

`print("Orientation :\n", orientation)`

### Matrice homogène totale
`T06 = robot.get_matrice_homogene()`

`print("Matrice homogène T06 :\n", T06)`

# Méthodes principales

| Méthode                  | Description                                     |
| ------------------------ | ----------------------------------------------- |
| `get_position()`         | Retourne la position cartésienne de l’effecteur |
| `get_oriontation()`      | Retourne la matrice de rotation de l’effecteur  |
| `get_matrice_homogene()` | Retourne la matrice homogène complète (T_0^6)   |
| `set_angles(q)`          | Met à jour les angles articulaires              |

## ✅ Remarque

Les angles articulaires doivent être exprimés en radians.
La configuration géométrique du robot est définie dans le fichier `configuration.py`.