# Rapport de Recherche — Fleet Orchestration (EDTH Paris 2026)

Ce journal trace **chaque itération expérimentale** du solveur dans `fleet-kit/handin/`, avec :
- **Version / Commit Git** permettant de revenir à tout état antérieur.
- **Intuition & hypothèses** (physique, mathématiques, diagnostics d'erreurs).
- **Modifications précises** apportées au code.
- **Résultats quantitatifs** (validation set `L1-101..L4-402`, temps d'exécution, validité R1–R10, et benchmarks hors-distribution sur seeds jamais vus).

---

## Tableau Récapitulatif Global (Validation Set 8 scénarios)

| Version | Commit | L1-101 | L2-201 | L2-202 | L3-301 | L3-302 | L3-303 | L4-401 | L4-402 | **TOTAL** | **Ratio Base** | Max Time | Valid |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| Baseline | `d9da7dd` | 68.0 | 277.8 | 232.9 | 423.8 | 326.5 | 275.9 | 641.2 | 418.5 | **2664.5** | 1.000× | 0.2s | 8/8 |
| V1 | *(pré-git)* | 72.5 | 299.9 | 267.3 | 728.9 | 423.0 | 455.8 | 1664.1 | 1137.9 | **5049.4** | 1.895× | 80.5s (KO) | 8/8 |
| V2 | *(pré-git)* | 72.5 | 299.9 | 267.3 | 729.1 | 423.0 | 453.0 | 1700.4 | 1186.1 | **5131.3** | 1.926× | 24.5s | 8/8 |
| V3 | `cb99e45` | 72.5 | 299.3 | 267.3 | 729.1 | 423.0 | 453.0 | 1688.4 | **1244.0** | **5176.7** | **1.943×** | 39.9s | 8/8 |
| V4 | `5a7c489` | 72.5 | **299.9** | 267.3 | 729.1 | 423.0 | **455.8** | 1693.7 | 1217.1 | 5158.3 | 1.936× | 37.1s | 8/8 |
| **V5** | *(courant)* | **72.5** | **299.9** | **267.3** | **729.1** | **423.0** | **455.8** | 1694.7 | 1223.9 | 5166.1 | 1.939× | **18.98s** | 8/8 |

---

## Expérience V1 — Architecture Fondamentale (2D Knapsack + Hungarian + Disk-TSP ALNS + 3D Deconfliction)

### Intuition physique & mathématique
L'analyse complète de `fleetkit` (`energy.py`, `validator.py`, `scorer.py`, `geometry.py`) a révélé plusieurs leviers majeurs inexploités par la baseline gloutonne :
1. **Vitesse optimale `v* = sqrt(p0 / k_v)`** : pour quasi tous les effecteurs, `v* >= v_h_mps`, donc voler à vitesse maximale `v_h_mps` est strictement optimal en énergie. Pour les scouts, l'énergie de croisière est strictement convexe en durée de segment : ralentir uniformément sur le slack disponible est toujours strictement moins cher que voler vite et stationner.
2. **Montée/descente diagonale & vol direct `z=0` pour les effecteurs** : R2 autorise la montée/descente en diagonale sur la première et dernière jambe (évite le coût temps+énergie de la montée verticale au pad), et `low = 0.0` sur le dernier waypoint `k == len(w) - 1`, permettant `[home(z=0), engage(z=0)]` sans montée quand le corridor est dégagé (~19 pts économisés en L4).
3. **Disk-TSP** : à `z = 30 m`, le rayon horizontal d'observation est `sqrt(150² - 30²) = 146.97 m`. Des contacts proches (<294 m) ont des disques qui se chevauchent (0 m de vol entre eux).
4. **Hétérogénéité de la flotte** : jusqu'à 60% d'écart en `Wh/m` entre le meilleur et le pire aéronef. Un appariement exact (algorithme Hongrois rectangulaire `O(n·m²)`) des effecteurs aux cibles et des scouts aux tournées économise des centaines de Wh.
5. **Décomposition couplée** : sélection du sous-ensemble optimal d'hostiles par Knapsack 2D exact (capacités `light`, `heavy`) + tarification itérative des coûts marginaux scouts/effecteurs + ALNS/VND pour le VRPTW des scouts + moteur de déconfliction 3D R8 indexé par AABB.

### Résultats V1
- **Total validation : 5049.4** (**1.895× baseline**), 8/8 valides.
- **L3-301 : 1007/1007 pts servis (100%)**.
- **Problèmes identifiés** :
  - **Timeout sur `L4-402` (80.51 s > 60 s)** : 85 effecteurs sur des pads distincts provoquaient 4746 cache-misses dans le routage 2D Dijkstra (polygones 24 côtés), prenant 39.57 s rien qu'à l'init.
  - **Perte en déconfliction sur `L4-402` (-81 pts)** : 4 scouts lancés à `t0 = 0.0` côte à côte (`11.1 m` entre pads) échouaient au décollage car un décalage uniforme violait les fenêtres tendues ou la batterie.

---

## Expérience V2 — Accélération 67× du Routage 2D + Déconfliction ciblée au décollage

### Intuition
1. **Cache spatial grossier 160 m + A* + polygones 16 côtés (`geometry3d.py`)** : tous les pads étant regroupés dans une boîte 100×100 m, les sommets intermédiaires de contournement d'obstacles sont quasi identiques pour tous les effecteurs vers une même cible. Réutiliser les sommets intermédiaires + shortcutting exact élimine les cache-misses.
2. **Retard de décollage seul (`min_t0`) et avance d'atterrissage seule (`max_t_land`) dans `timing.py`** : décaler uniquement `t0` de quelques secondes (attente gratuite au sol, non comptée en R8) résout les conflits de pad à `t = 0..10 s` sans déplacer les fenêtres d'observation serrées, pour ~0.01 Wh.
3. **Substitution par scout libre et découpage de tournée** dans `deconflict.py` / `solver.py` si un scout échoue encore.

### Résultats V2
- **Init `L4-402`** : **39.57 s → 0.59 s (67× plus rapide !)**.
- **Temps total `L4-402`** : **80.51 s → 15.50 s** ; `L4-401` : **40.57 s → 24.48 s**.
- **Score total : 5049.4 → 5131.3 (+81.9 pts, 1.926× baseline)**, 8/8 valides :
  - `L4-401` : **1664.1 → 1700.4 (+36.3 pts)**
  - `L4-402` : **1137.9 → 1186.1 (+48.2 pts)**
  - Régression mineure sur `L3-303` : `455.8 → 453.0 (-2.8 pts)` due au buffer `stagger_buf` trop conservateur.

### Diagnostics effectués sur V2 (révélations majeures pour V3)
1. **Bug des scouts libres (`empty_scouts[0]`)** : sur `L4-402`, 2 unknowns rentables même en solo (`C62` : +11 pts, `C66` : +13 pts) n'étaient pas servis car `empty_scouts[0]` était un petit scout (26.8 Wh) incapable d'atteindre 36 Wh, alors que `S05` (49.1 Wh) ou `S11` (43.6 Wh) étaient libres !
2. **Ordre de placement des scouts en déconfliction** : `S07` (33.1/33.2 Wh, 0.1 Wh de marge) et `S09` (39.6/40.0 Wh, 0.4 Wh de marge) étaient placés après des scouts à grosse marge batterie et échouaient en déconfliction (-72 pts sur `L4-402`).
3. **Conflit `C11` vs `S15` sur `L4-401` (-19.6 pts)** :
   - `build_effector_route(..., z_cruise > 0)` ne plaçait pas de waypoint de montée rapide près de la base → l'effecteur montait en rampe douce sur 2.5 km et restait à `z < 10 m` dans le corridor d'atterrissage de `S15`.
   - `_try_dodge_chain` sur effecteurs (Tier 4) tournait avec `extra_duration=0.0` (vitesse max `v_h_mps` = 0 s de slack), faisant toujours échouer `_insert_local_dodge`.
4. **Tarification effecteurs inactifs** : `_compute_true_marginals` utilisait `est_eff_e` (meilleurs effecteurs globaux même déjà occupés) au lieu des **potentiels duaux exacts du Hongrois** (`u, v`).

---

## Expérience V3 — Potentiels Duaux Hongrois + Scouts Libres Diversifiés + Early-Climb Effectors + Or-opt-2

### Modifications testées
1. `assignment.py` : ajout de `hungarian_min_cost_duals` retournant `(total, col_for_row, row_u, col_v)` exacts.
2. `solver.py` :
   - `_candidate_empty_scouts` exposant jusqu'à 3 scouts libres diversifiés (plus économe, **plus grosse batterie**, plus rapide) dans `_regret2_construct`, `_vnd_scout_routes` et `_compute_true_marginals`.
   - Utilisation des potentiels duaux Hongrois (`u_by_cid`, `v_light`, `v_heavy`) dans `_compute_true_marginals`.
   - Tri des scouts en déconfliction par **marge batterie croissante** (`usable_wh - tour_e`) + rip-up & re-order + smart split sur tous les points de coupe.
   - Rescue effecteurs : 3 combinaisons d'effecteurs libres + avance d'observation scout si fenêtre serrée.
   - Ajout de **Or-opt-2** (intra-route et inter-route) dans `_vnd_scout_routes`.
3. `timing.py` :
   - Réduction de `stagger_buf` sur les fenêtres hostiles (grâce à l'exemption R8 dans les 60 m de la cible).
   - Waypoint de montée rapide `c_pt` près du pad dans `build_effector_route` quand `z_cruise > 0`.
4. `deconflict.py` :
   - Pas de décalage plus fins pour scouts (`1.5 s, 3.0 s, ...`).
   - Tier 4 effecteurs avec `extra_duration = 6.0 / 14.0` pour donner du slack réel à `_try_dodge_chain`.

### Résultats V3 (Validation Set)
- **Total : 5131.3 → 5176.7 (+45.4 pts, 1.943× baseline)**, 8/8 valides :
  - `L1-101` : 72.5 (0.00 s)
  - `L2-201` : 299.3 (0.18 s) *(-0.6 pts vs V2)*
  - `L2-202` : 267.3 (0.27 s)
  - `L3-301` : 729.1 (2.17 s)
  - `L3-302` : 423.0 (1.32 s)
  - `L3-303` : 453.0 (2.70 s)
  - `L4-401` : **1688.4 (39.87 s)** *(-12.0 pts et +15.4 s vs V2 !)*
  - `L4-402` : **1244.0 (29.07 s)** *(**+57.9 pts** vs V2, **+106.1 pts** vs V1 !)*

### Analyse critique de V3 (pourquoi `L4-401` a ralenti et baissé de 12 pts)
- **Succès massif sur `L4-402` (+57.9 pts)** : les corrections sur les scouts libres grande-batterie + tri par marge batterie + rip-up + early-climb ont fonctionné exactement comme prévu.
- **Cause de la régression sur `L4-401` (24.5 s → 39.9 s, 1700.4 → 1688.4)** : l'ajout de **Or-opt-2 inter-route (`2b`)** à CHAQUE passe de `_vnd_scout_routes` (appelé dans chaque itération ALNS et chaque swap hostile) a multiplié par ~3 le coût de VND sur L4 (`~12 000` évaluations par passe), faisant expirer `search_deadline` trop tôt et privant Stages 2/3/4/6 de budget d'exploration !
- **Piste supplémentaire identifiée sur `L3-303`** : lorsque le Hongrois affecte les tournées aux scouts, le scout le plus économe (`Wh/m`) mais à petite batterie (`S04`, 29.1 Wh) absorbe la grosse tournée et sature sa batterie, empêchant de fusionner 3 tournées en 2 tournées (alors qu'un scout à grande batterie `S03`, 44.1 Wh + `S05`, 33.9 Wh permettrait de réduire le nombre de scouts et d'économiser ~6 Wh).

### Prochaine itération planifiée : V4
1. Retirer `Or-opt-2` inter-route des passes VND rapides (le réserver uniquement au polish final Stage 4, ou le filtrer spatialement aux paires très proches `< 400 m`) → retrouver la vitesse <20 s sur L4 et débloquer tout le budget de recherche Stages 2/3/4/6 !
2. Tester le **Tour Merge (fusion explicite de 2 tournées sur un scout à grande batterie)** dans VND/Stage 4 pour réduire le nombre de scouts utilisés quand rentable (`L3-303` et `L4`).
3. Diagnostiquer la micro-régression de `-0.6 pts` sur `L2-201` (probablement ordre shifts vs delays en déconfliction).
4. Lancer un benchmark **hors-distribution multi-seeds** (`L1–L4, seeds 0-9`, soit 40 scénarios inédits) pour valider la généralisation !

---

## Expérience V4 — Tour Merge + Gating Or-opt-2 (`deep=True`) + Restauration `stagger_buf`

### Intuition
1. **Correction `L2-201` (`299.3 → 299.9`)** : le diagnostic des routes de `L2-201` a montré que `H01` grimpait à `z = 30 m` (+1.2 Wh = -0.6 pts) parce que la réduction de `stagger_buf` en V3 (1.5 s max) ne laissait que 1.5 s d'écart entre `H03` et `H01`, provoquant un rattrapage en vol. Restaurer `stagger_buf = min(2.0 + 3.0*(n_eff-1), 0.08*avail)` préserve le vol `z = 0` sans surcoût.
2. **Tour Merge (`3b` dans VND)** : sur `L3-303`, l'appariement Hongrois plaçait la grosse tournée sur un scout économe à petite batterie (`S04`, 29.1 Wh), bloquant la fusion d'une petite tournée de 2 contacts. Un opérateur explicite de fusion de deux tournées sur un scout grande-batterie doit éliminer des trajets base↔terrain.
3. **Gating de `Or-opt-2` inter-route** : réservé au polish final `deep=True` (Stage 4) et filtré aux paires proches (`c_dist <= 600 m`) pour regagner la vitesse V2 sur L4.

### Résultats V4 (Validation Set)
- **Total : 5158.3** (**1.936× baseline**), 8/8 valides :
  - `L1-101` : **72.5** (0.01 s)
  - `L2-201` : **299.9** (0.18 s) — **+0.6 pts** (exactement comme prédit !)
  - `L2-202` : **267.3** (0.29 s)
  - `L3-301` : **729.1** (3.65 s)
  - `L3-302` : **423.0** (0.97 s)
  - `L3-303` : **455.8** (2.27 s) — **+2.8 pts (-5.5 Wh)** grâce à Tour Merge !
  - `L4-401` : **1693.7** (37.08 s) — **+5.3 pts** vs V3, mais encore 37.1 s (vs 24.5 s en V2)
  - `L4-402` : **1217.1** (28.32 s) — **-26.9 pts** vs V3 (1244.0) !

### Analyse scientifique (ablation nécessaire pour V5)
- **Victoire totale sur L1–L3** : tous les 6 scénarios L1–L3 égalent ou battent leur meilleur score historique (`72.5 + 299.9 + 267.3 + 729.1 + 423.0 + 455.8 = 2247.6`).
- **Deux points à isoler immédiatement par ablation** :
  1. Pourquoi `L4-402` a perdu 26.9 pts entre V3 et V4 ? Hypothèse : soit `stagger_buf` (0.08×avail = jusqu'à 11 s retirées aux fenêtres serrées de L4-402), soit `Tour Merge` qui a consommé du budget CPU dans VND.
  2. Pourquoi `L4-401` prend encore 37 s (vs 24.5 s en V2) ? Hypothèse : `Tour Merge (3b)` + `Or-opt-2` intra-route tournent à CHAQUE passe de VND interne (dans chaque itération ALNS et chaque swap hostile), + `_candidate_empty_scouts` évalue 3 scouts vides au lieu de 1 dans les boucles internes.

---

## Expérience V5 — Profilage `cProfile` & Accélération 2–3× du cœur d'évaluation VRPTW

### Diagnostic `cProfile` sur V4 (L4-401 & L4-402)
Le profilage détaillé de V4 a révélé que **`eval_scout_tour` représentait 89% du temps sur `L4-401` (17.75 s / 19.93 s)** et **71% sur `L4-402` (17.59 s / 24.84 s)** :
1. **`optimize_disk_tour` + `_best_point_in_disk` + `point_inside_any` (9.32 s sur L4-401)** : 4.24 M appels à la closure golden-section `f` et 577 K appels à `point_inside_any` vérifiant les 28 cylindres alors que >90% des disques de contacts ne touchent 0 cylindre !
2. **`best_path_3d` / `shortest_path_2d` non cachés (~8.5 s)** : 51 247 appels redondants à `shortest_path_2d` sur `L4-402` (6.32 M appels à `segment_hits_cyl_2d`).
3. **`build_scout_route` dans la boucle interne VRPTW (~2 s)** : allouait des dicts JSON, arrondissait et re-parsait `_to_waypoints` à chaque évaluation de mouvement VND/ALNS.
4. **Tour Merge `3b` exécuté à chaque passe interne de VND** au lieu d'être exécuté uniquement à l'optimum local VND.

### Modifications implémentées (V5)
1. `geometry3d.py` :
   - Cache `_disk_blocked_indices(contact, z)` évitant `point_inside_any` quand 0 obstacle n'intersecte le disque.
   - Cache `_leg_cands_cache` (quantifié à 0.5 m, indépendant de l'aéronef) dans `best_path_3d`.
   - **`_best_point_in_disk` analytique** : réflexion de Fermat + 1 itération de point fixe exacte (`< 0.01 m`, zéro closure/trigo) quand `da > 1.35 r, db > 1.35 r, un > 0.25`, avec fallback golden-section inliné (10 itérations).
2. `timing.py` :
   - Sortie anticipée (early-exit) dans `optimize_scout_timing` dès que l'initialisation Step 3 atteint `ideal_dt` sur toutes les jambes.
   - **`eval_scout_route_energy`** : évaluateur exact d'énergie scout sans allocation JSON / `_to_waypoints` (parité vérifiée sur 180 tournées L3/L4 : **0 mismatch, écart max = `0.000000 Wh`**).
   - `stagger_buf` adaptatif `min(1.5 + 1.5*(n_eff-1), max(0.25, 0.05*avail))` avec **fallback automatique à `0.25 s`** si la fenêtre serrée échoue.
3. `solver.py` :
   - Câblage de `eval_scout_route_energy` dans `eval_scout_tour` + `n_iters=3` dans `_get_disk_pts` (le final `deconflict.py` garde `n_iters=8` avec le vrai `ac.home`).
   - Gating de `Or-opt-2` intra-route à `deep or n <= 8`.
   - Déplacement de **Tour Merge** à l'optimum local VND avec réaffectation exacte `_try_hungarian_scout_match` sur les `k - 1` tournées (permettant à un scout grande batterie déjà occupé d'absorber la fusion pendant qu'un autre récupère son ancienne tournée).

### Résultats V5 (Validation Set)
- **Total : 5166.1 (+7.8 pts vs V4, 1.939× baseline)**, **8/8 valides** :
  - `L1-101` : **72.5** (`0.00 s`)
  - `L2-201` : **299.9** (`0.12 s`)
  - `L2-202` : **267.3** (`0.19 s`)
  - `L3-301` : **729.1** (`1.25 s` vs 3.65 s en V4 — **2.9× plus rapide**)
  - `L3-302` : **423.0** (`0.54 s` vs 0.97 s en V4 — **1.8× plus rapide**)
  - `L3-303` : **455.8** (`0.76 s` vs 2.27 s en V4 — **3.0× plus rapide**)
  - `L4-401` : **1694.7** (`18.98 s` vs 37.08 s en V4 — **2.0× plus rapide**, +1.0 pt)
  - `L4-402` : **1223.9** (`15.97 s` vs 28.32 s en V4 — **1.8× plus rapide**, +6.8 pts)
- **Temps max divisé par 2** : **18.98 s** (marge de **>41 s** sous la limite de 60 s !). Cela nous libère un énorme budget CPU pour pousser la qualité d'optimisation bien plus loin en V6+.
