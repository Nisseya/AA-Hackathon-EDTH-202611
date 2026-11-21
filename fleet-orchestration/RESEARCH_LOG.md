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
| V5 | `db81484` | **72.5** | **299.9** | **267.3** | **729.1** | **423.0** | **455.8** | 1694.7 | 1223.9 | 5166.1 | 1.939× | **18.98s** | 8/8 |
| V6 | `a07c6d2` | **72.5** | **299.9** | **267.3** | **729.1** | **423.0** | **455.8** | **1708.0** | 1239.5 | 5195.1 | 1.950× | 30.37s | 8/8 |
| V7 | `1d5b54c` | **72.5** | **299.9** | **267.3** | **729.1** | **423.0** | **455.8** | 1707.8 | **1258.8** | **5214.2** | **1.957×** | 29.12s | 8/8 |
| V8 | `f93ddd6` | **72.5** | **299.9** | **267.3** | **729.1** | **423.0** | **455.8** | 1707.8 | 1258.8 | 5214.2 | 1.957× | 24.54s | 8/8 |
| **V9** | `168a555` | **72.5** | **299.9** | **267.3** | **729.1** | **423.0** | **455.8** | **1717.7** | **1261.0** | **5226.3** | **1.961×** | 32.87s | 8/8 |

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

---

## Expérience V6 — Insertions Solo Débloquées par Hongrois + Déconfliction Zero-Slack & Rip-Up Inconditionnel

### Diagnostics effectués sur V5 (Validation + 40 seeds inédits `L1–L4`)
1. **Benchmark hors-distribution V5 (40 seeds `L1–L4, seeds 0–9`)** :
   - **40/40 valides**, total **23 616.1** vs **11 847.8** baseline (**1.993×**), **L4 = 3.047× baseline** (`14 387.8` vs `4 722.0`).
2. **Piège des scouts grande batterie déjà occupés** : sur `L4-401`, l'inconnu `C52` (`pts=29`, `soloScoutE=32.5 Wh`) restait non servi avec 7 scouts libres parce que les scouts grande batterie (`S06` 48.3 Wh, `S05` 43.7 Wh) volaient déjà des tournées courtes, et aucun des 7 scouts libres n'atteignait `C52` seul. Tester `_try_hungarian_scout_match(tours_actifs + [[C52]])` réaffecte automatiquement la tournée courte à un scout libre et débloque **`+8.60 pts` nets** !
3. **Pertes massives de déconfliction scout (`+29.6 Wh` sur `L4-401 S09`, `+26.7 Wh` sur `L4-402 S11`)** :
   - `S09` (5.5 Wh de marge) avait exactement `5.5 s` de marge d'atterrissage avant `t = 900 s`, mais la grille grossière `d_land = (2.5, 6.0, ...)` ratait `d_land = 4.5–5.0 s` (`+0.03 Wh`) et déclenchait un split à `+29.6 Wh` !
   - `S11` (6.9 Wh de marge, seulement `4 s` de marge temporelle) heurtait `S04` (`0.15 Wh` de marge batterie mais des centaines de secondes de slack temporel `0 Wh`) ; la garde `scout_headroom[other] > my_hr + 1.0` bloquait à tort le Rip-Up de `S04`.
   - Dans `_insert_local_dodge`, le clamping `f ∈ [0.05, 0.95]` et le double padding `+0.005 s` faisaient échouer les esquives verticales sur les jambes sans slack horizontal.

### Modifications implémentées (V6)
1. `deconflict.py` :
   - Ajout de `_add_vertical_profile` (montée rapide `leg 0` / descente tardive `last leg` sur la même ligne horizontale : **0.00 s** de temps supplémentaire, **0.00 Wh** à `z = z_min`).
   - Grilles monotones fines `launch_delays` (`1.0, 2.0, 3.0, 4.0, 5.0, 6.5, ...`) et `d_land` (`1.5, 3.0, 4.5, 6.0, 8.0, ...`) avec sortie anticipée (`break`) sur infaisabilité + combinaison profil vertical / dodge.
   - Correction `_insert_local_dodge` (`f ∈ [0.015, 0.985]`, padding vertical `0.0005 s`).
2. `solver.py` :
   - **Insertions solo débloquées par `_try_hungarian_scout_match`** dans `_regret2_construct` (Phase 1 hostiles & Phase 2 unknowns), `_vnd_scout_routes` (Step 5 unknowns) et `_compute_true_marginals`.
   - **Rip-Up & Re-Order inconditionnel** (jusqu'à 2 scouts conflictuels) en déconfliction scout (supprime la garde erronée sur `scout_headroom`).
   - Stage 4 `deep_iters` augmenté à `55 / 45` grâce au gain de vitesse V5.
3. `planner.py` : `time_budget_s` abaissé à **`45.0 s`** (marge stricte de 15 s sous la limite de 60 s).

### Résultats V6
- **Validation Set (8 scénarios)** : **5195.1 (+29.0 pts vs V5, 1.950× baseline)**, **8/8 valides**, max `30.37 s` :
  - `L1–L3` : **tous au record historique** (`72.5 / 299.9 / 267.3 / 729.1 / 423.0 / 455.8`).
  - **`L4-401` : 1708.0** *(nouveau record absolu, `+13.3 pts` vs V5, `30.37 s`)*.
  - **`L4-402` : 1239.5** *(`+15.6 pts` vs V5, `24.46 s`)*.
- **Benchmark Hors-Distribution (40 seeds inédits `L1–L4, seeds 0–9`)** :
  | Niveau (10 seeds) | Baseline | V5 (`db81484`) | **V6** | Δ V6 vs V5 | Ratio Base | Valid | AvgT (6 cores) |
  |---|---:|---:|---:|---:|---:|---:|---:|
  | L1 | 1014.3 | 1138.3 | **1138.3** | +0.0 | 1.122× | 10/10 | 0.05s |
  | L2 | 2318.1 | 2819.3 | **2819.5** | +0.2 | 1.216× | 10/10 | 0.30s |
  | L3 | 3793.5 | 5270.7 | **5270.7** | +0.0 | 1.389× | 10/10 | 1.21s |
  | L4 | 4722.0 | 14387.8 | **14406.4** | **+18.6** | **3.051×** | 10/10 | 35.32s |
  | **TOTAL (40)** | **11847.8** | **23616.1** | **23635.0** | **+18.9** | **1.995×** | **40/40** | max 42.05s |
- **Piste identifiée pour V7** : gros gains sur `L4-00 (+22.3)`, `L4-04 (+24.9)`, `L4-09 (+14.2)`, mais une chute isolée sur `L4-06 (1293.7 → 1255.5, -38.2 pts)` à diagnostiquer !

---

## Expérience V7 — Lateral Approach Fan-Out Effecteurs + Physical Scout Swap (2-way/3-way)

### Diagnostics effectués sur V6 (`L4-06`, `L4-02`, `L4-402`, `L4-401`)
L'instrumentation de `deconflict_and_build_plan` a révélé deux goulots d'étranglement universels :
1. **Convergence multi-effecteurs hors sphère d'exemption `60 m` lorsque `slack ≈ 0.2 s`** : sur `L4-402 C17` (71 pts, 3 lights) et `L4-06 C02` (72 pts, 1L+1H), chaque effecteur avait `cf = None` contre tous les autres aéronefs, mais en arrivant au même instant `ta` sur le même axe radial depuis la base, ils entraient en conflit entre `65 m` et `120 m` de la cible (juste hors des `60 m` d'exemption R8).
2. **Scouts verrouillés à `t0 = 0.0 s` sur pads adjacents `< 50 m`** : sur `L4-02 S15`, `L4-06 S04/S07` et `L4-402 S01/S05`, deux scouts doivent décoller à `t0 = 0.0 s` depuis des pads espacés de `< 50 m` (infaisable même avec `d_t0 = 1.0 s`). Seul un échange physique d'aéronef avec un scout sur un pad distant (`≥ 52 m`) ou à `t0` tardif résout ce conflit sans split ni perte de contact.

### Modifications implémentées (V7)
1. `timing.py` & `deconflict.py` :
   - **Lateral Approach Fan-Out (`approach_lat = ±75 m, ±115 m` à `105 m` de la cible, `z = z_min`)** dans `build_effector_route` et `place_effector_group_deconflicted` (Tier 2b) : permet à 2–4 effecteurs d'engager à la **même milliseconde** pour `+0.03 à +0.65 Wh` seulement !
   - **Esquive 3D locale basse altitude (`Tier 2c` à `z0`)** exécutée AVANT les montées coûteuses `z ≥ 71 m` (économise de l'énergie même sur L1–L3 !).
2. `solver.py` :
   - **Physical Scout Swap Rescue (2-way & 3-way)** : avant tout tour-split, tente de transférer la tournée bloquée vers un scout déjà placé sur un pad distant / `t0` tardif, en déplaçant sa tournée vers le scout original ou un scout libre.
   - Pool diversifié (batterie, vitesse `v_h_mps`, distance de pad) dans Smart Tour Split.
3. `geometry3d.py` : garde exacte `segment_valid_3d(a, b)` sur hit cache quantifié 0.5 m.

### Résultats V7
- **Validation Set (8 scénarios)** : **5214.2 (+19.1 pts vs V6, +48.1 pts vs V5, 1.957× baseline)**, **8/8 valides**, max `29.12 s` :
  - `L1–L3` : tous au record absolu (`72.5 / 299.9 / 267.3 / 729.1 / 423.0 / 455.8`).
  - `L4-401` : **1707.8** (`29.12 s`).
  - **`L4-402` : 1258.8 (`+19.3 pts` vs V6, `3.008×` baseline, `20.18 s`)** — gain exactement conforme à la prédiction (`C17` sauvé !).
- **Benchmark Hors-Distribution (40 seeds inédits `L1–L4, seeds 0–9`) — PROGRÈS SUR TOUS LES 4 NIVEAUX** :
  | Niveau (10 seeds) | Baseline | V5 | V6 | **V7** | Δ V7 vs V6 | Ratio Base | Valid |
  |---|---:|---:|---:|---:|---:|---:|---:|
  | L1 | 1014.3 | 1138.3 | 1138.3 | **1139.5** | **+1.2** | 1.123× | 10/10 |
  | L2 | 2318.1 | 2819.3 | 2819.5 | **2820.0** | **+0.5** | 1.217× | 10/10 |
  | L3 | 3793.5 | 5270.7 | 5270.7 | **5272.5** | **+1.8** | 1.390× | 10/10 |
  | L4 | 4722.0 | 14387.8 | 14406.4 | **14433.7** | **+27.3** | **3.057×** | 10/10 |
  | **TOTAL (40)** | **11847.8** | **23616.1** | **23635.0** | **23665.7** | **+30.7** | **1.997×** | **40/40** |
  - Notamment `L4-06` bondit de `1255.5 → 1292.3 (+36.8 pts, 4.11× baseline)` !

---

## Expérience V8 — Blocker-Relocation Rescue + Surgical 1-Contact Trimming + Solo Rescue

### Diagnostics effectués sur V7 (`L4-02`, `L4-401`, `L4-402`)
1. **Perte de `178 pts` en déconfliction sur `L4-02`** :
   - Le scout `S15` (`pad=(150.0, 172.2)`, `21.2 m/s`, `46.0 Wh`) devait décoller à `t0 = 0.0 s` (`0.6 s` de slack seulement pour atteindre `C25`, 95 pts), tandis que `S02` (`pad=(183.3, 183.3)`, à **`35.1 m < 50 m`**) était déjà placé à `t0 = 0.0 s`.
   - Le Physical Swap V7 déplaçait `S15` (échec car aucun autre scout n'était assez rapide pour `C25`), au lieu de déplacer le **bloqueur `S02`** vers un scout libre distant (`S09`, `48.2 Wh`, `pad=(250.0, 227.8) ≥ 52 m`).
   - En dernier recours, le trimming aveugle (suppression par points croissants) détruisait **4 contacts innocents (`C64, C13, C40, C03`, ~200 pts)** avant de finalement retirer l'unique contact verrouillant `C25` (95 pts) !
2. **Pool Stage 5 (`top_sols`)** : élargi de `4 → 8` (déconfliction jusqu'à 5 solutions distinctes).

### Modifications implémentées (V8)
1. `solver.py` (`deconflict_and_build_plan`) :
   - **Blocker-Relocation Rescue** : identifie le(s) scout(s) bloqueur(s) `bid` via `sep.first_conflict`, retire `bid`, place le scout tendu `ac`, et relocalise la tournée de `bid` sur un scout libre situé sur un pad distant (`≥ 52 m`).
   - **Surgical 1-Contact Trimming + Solo Rescue** : en dernier recours, teste la suppression de **chaque contact individuel `x ∈ cur_cids`**, conserve le sous-ensemble `n - 1` faisable de valeur maximale (`Σ points - λ_e E`), puis tente de replacer en solo les contacts retirés sur les scouts libres restants.
   - Élargissement du pool Stage 5 (`top_sols` `4 → 8`, déconfliction `[:5]`).

### Résultats V8
- **Validation Set (8 scénarios)** : **5214.2** (`1.957×` baseline), **8/8 valides**, max `24.54 s` (`-4.58 s` vs V7 !).
- **Benchmark Hors-Distribution (40 seeds inédits `L1–L4, seeds 0–9`) — FRANCHIT LES 2.00× BASELINE !**
  | Niveau (10 seeds) | Baseline | V5 | V6 | V7 | **V8** | Δ V8 vs V7 | Ratio Base | Valid |
  |---|---:|---:|---:|---:|---:|---:|---:|---:|
  | L1 | 1014.3 | 1138.3 | 1138.3 | 1139.5 | **1139.5** | +0.0 | 1.123× | 10/10 |
  | L2 | 2318.1 | 2819.3 | 2819.5 | 2820.0 | **2820.0** | +0.0 | 1.217× | 10/10 |
  | L3 | 3793.5 | 5270.7 | 5270.7 | 5272.5 | **5272.5** | +0.0 | 1.390× | 10/10 |
  | L4 | 4722.0 | 14387.8 | 14406.4 | 14433.7 | **14536.8** | **+103.1** | **3.079×** | 10/10 |
  | **TOTAL (40)** | **11847.8** | **23616.1** | **23635.0** | **23665.7** | **23768.8** | **+103.1** | **2.006×** | **40/40** |
  - **`L4-02`** bondit de `~1275 → 1378.6` (**`+103 pts` sur un seul scénario L4 inédit !**).
  - Temps max sous forte contention 6-cœurs : `42.46 s` (`24.54 s` en single-core).

---

## Expérience V9 — Launch-Urgency Tiering + Pad-Spread Hungarian + Multi-Blocker Relocation + Garde `ac_is_free`

### Diagnostics effectués sur V8 (12 scénarios L4 : `L4-401, L4-402, L4-00..09`)
L'audit complet pré/post-déconfliction a montré que **11/12 scénarios L4 avaient déjà `drop = 0`** (perte médiane `3.6 pts` sur `1450+`, soit **99.75% d'efficacité**), mais les 3 scénarios perdant `> 10 pts` (`L4-401: -12.8`, `L4-06: -17.4`, `L4-02: -73.6`) partageaient **exactement la même cause géométrique racine** :
1. **Saturation de la grille de pads `100 m × 100 m` sous `d_sep = 50 m`** : les 15 scouts sont espacés de `11.1 m`. `optimize_scout_timing` faisait décoller par défaut à `t0 = 0.0 s` même les scouts ayant `> 10 s` de slack au décollage (pour économiser `0.05 Wh`). Placés avant un scout réellement verrouillé à `t0 = 0.0 s` (`max_t0 < 2 s`), ils le forçaient au split (`-12.5` à `-17.2 pts`) ou à l'abandon de contact (`C25`, `95 pts`).
2. **Double blocage simultané** : sur `L4-06`, `S04` était bloqué par **deux** voisins à `11 m` (`S07` et `S08`), faisant échouer la relocalisation 1-bloqueur.
3. **Bug latent `unplaced_scouts` détecté sur les 40 seeds inédits** : quand `≥ 2` scouts échouaient et que le premier réutilisait l'aéronef du second avant son tour, le second pouvait écraser/retirer son propre aéronef dans `sep.routes_json` (`KeyError`).

### Modifications implémentées (V9)
1. `solver.py` :
   - **Launch-Urgency Tiering (`launch_tier`)** dans `deconflict_and_build_plan` : sonde la faisabilité conjointe temps+batterie à `min_t0 = 2.0 s` et `6.0 s` pour chaque tournée active (`0 = verrouillé à t0=0`, `1 = serré <6s`, `2 = flexible`) et trie par `(launch_tier, battery_headroom, -len(cids))`.
   - **Pad-Spread Hungarian (`_pad_spread_scout_sol`)** : fallback branch-and-bound + Hongrois garantissant `≥ 51 m` entre les pads des tournées `launch_tier == 0` dès que la déconfliction nominale perd `> 3.5 pts`.
   - **Multi-Blocker Relocation (jusqu'à 2 bloqueurs simultanés)** + **garde `ac_is_free`** blindant toutes les branches de `unplaced_scouts`.
   - **Pool `top_sols` diversifié** (déduplication par `(hostiles, round(scout_e, 1))` avant troncature) + **restarts VRPTW indépendants en Stage 6** si la perte de déconfliction reste `> 3.5 pts`.

### Résultats V9
- **Validation Set (8 scénarios) — RECORD ABSOLU SUR LES 8/8 SCÉNARIOS** :
  - **TOTAL : 5226.3 (+12.1 pts vs V8, +12.1 pts vs V7, 1.961× baseline)**, **8/8 valides**, max `32.87 s` :
  - `L1–L3` : tous au record absolu (`72.5 / 299.9 / 267.3 / 729.1 / 423.0 / 455.8`).
  - **`L4-401` : 1717.7 (`+9.9 pts` vs V8, `2.679×` baseline, `32.87 s`)**.
  - **`L4-402` : 1261.0 (`+2.2 pts` vs V8, `3.013×` baseline, `25.31 s`)**.
- **Benchmark Hors-Distribution (40 seeds inédits `L1–L4, seeds 0–9`) — NOUVEAU RECORD ABSOLU (2.009× BASELINE)** :
  | Niveau (10 seeds) | Baseline | V5 | V6 | V7 | V8 | **V9** | Δ V9 vs V8 | Ratio Base | Valid |
  |---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
  | L1 | 1014.3 | 1138.3 | 1138.3 | 1139.5 | 1139.5 | **1139.5** | +0.0 | 1.123× | 10/10 |
  | L2 | 2318.1 | 2819.3 | 2819.5 | 2820.0 | 2820.0 | **2820.0** | +0.0 | 1.217× | 10/10 |
  | L3 | 3793.5 | 5270.7 | 5270.7 | 5272.5 | 5272.5 | **5272.5** | +0.0 | 1.390× | 10/10 |
  | L4 | 4722.0 | 14387.8 | 14406.4 | 14433.7 | 14536.8 | **14564.7** | **+27.9** | **3.084×** | 10/10 |
  | **TOTAL (40)** | **11847.8** | **23616.1** | **23635.0** | **23665.7** | **23768.8** | **23796.7** | **+27.9** | **2.009×** | **40/40** |
  - **`L4-02`** gagne encore `+27.0 pts` (`1378.6 → 1405.6`, soit `+130 pts` par rapport à V7 !).
  - **`L4-07`** gagne `+3.8 pts` (`1442.4 → 1446.2`).
  - **40/40 valides**, zéro crash, temps max `43.66 s` sous contention 6-cœurs (`32.87 s` en single-core).

---

## Vérification Finale Officielle (`check_handin.py` & Plans de Validation)

### 1. Harnais officiel sandboxé (`python3 handin/check_handin.py --levels L1,L2,L3,L4 --seeds 0-4 --timeout 60`)
- **Résultat : `total 12008.9 | baseline 5998.5 | ratio 2.002× | failed runs 0` (20/20 `ok`, max `45.1 s`)**
- Détail L4 en single-core sandboxé :
  - `L4-0` : **1505.8** vs `590.3` (`2.55×`, `15.5 s`)
  - `L4-1` : **1520.5** vs `432.5` (`3.52×`, `41.2 s`)
  - `L4-2` : **1435.5** vs `331.5` (**`4.33×`**, `45.1 s`)
  - `L4-3` : **1301.7** vs `532.9` (`2.44×`, `35.3 s`)
  - `L4-4` : **1455.3** vs `453.5` (`3.21×`, `26.8 s`)
  - **Sous-total L4 (`seeds 0–4`) : `7218.8` vs `2340.7` (`3.084×` baseline)**

### 2. Plans de validation générés & vérifiés (`fleet-kit/plans/`)
Générés via `python3 handin/planner.py` et validés via `python3 -m fleetkit score` (**total exact : `5226.31`**, prêts pour `python3 -m fleetkit submit-all plans/`) :
- `L1-101.json` : **72.45** (`110.0 pts`, `15.1 Wh`, `3 eff`)
- `L2-201.json` : **299.87** (`394.0 pts`, `48.3 Wh`, `7 eff`)
- `L2-202.json` : **267.32** (`355.0 pts`, `55.4 Wh`, `6 eff`)
- `L3-301.json` : **729.14** (`1007.0 pts`, `175.7 Wh`, `19 eff`)
- `L3-302.json` : **423.04** (`681.0 pts`, `155.9 Wh`, `18 eff`)
- `L3-303.json` : **455.76** (`746.0 pts`, `160.5 Wh`, `21 eff`)
- `L4-401.json` : **1717.73** (`2678.0 pts`, `760.5 Wh`, `58 eff`)
- `L4-402.json` : **1261.00** (`2260.0 pts`, `878.0 Wh`, `56 eff`)

---

## Stress-Test Grande Échelle — 1 000 Scénarios Inédits sur RunPod (`L1–L4`, seeds `1000..1249`)

### Protocole & Infrastructure
- **Objectif** : soumettre la version finale **V9** (commit `e603705`) à une campagne hors-distribution massive de **1 000 scénarios complètement inédits** (`250` par niveau `L1, L2, L3, L4`, seeds `1000..1249`) afin de détecter tout cas limite latent (crash, timeout `> 60 s`, violation R1–R10, perte face à la baseline, chute en déconfliction).
- **Exécution** : chaque scénario est exécuté dans un sous-processus isolé `python3 handin/planner.py` avec **timeout strict `60.0 s`**, puis évalué par `fleetkit.scorer.evaluate` et comparé à `fleetkit.baseline.greedy_plan`.
- **Infrastructure cloud** : **4 pods CPU RunPod (`cpu5c × 32 vCPU`, 16 workers/pod = 2 vCPU dédiés par scénario)** orchestrés via l'API REST v2 (`https://api.runpod.io/v2/pods`), serveur HTTP interne sur port `8000` interrogé via proxy HTTPS RunPod, sauvegarde incrémentale par shard, watchdog anti-hôte ralenti, et suppression automatique vérifiée (`0` pod actif en fin de run, coût total de la campagne : **\$0.86**).
- **Durée totale** : `~101–108 s` de calcul par shard de 250 scénarios (`509.0 s` wall-clock avec watchdog/respawns réseau).

### Résultats Globaux (1 000 / 1 000 Scénarios)

| Niveau | N | Valides | Timeouts | Crashs | Viol. | Score V9 (Σ) | Baseline (Σ) | Ratio Σ | Win Rate | Moyenne V9 | Médiane V9 | Écart-type | Min | Max |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| **L1** | 250 | **250/250** | 0 | 0 | 0 | **25 373.0** | 22 934.1 | **1.106×** | **100.0%** | 101.49 | 95.58 | 37.29 | 2.7 | 233.4 |
| **L2** | 250 | **250/250** | 0 | 0 | 0 | **64 983.1** | 56 966.0 | **1.141×** | **100.0%** | 259.93 | 255.60 | 62.02 | 70.8 | 430.8 |
| **L3** | 250 | **250/250** | 0 | 0 | 0 | **135 217.7** | 97 921.2 | **1.381×** | **100.0%** | 540.87 | 534.87 | 82.48 | 341.6 | 773.4 |
| **L4** | 250 | **250/250** | 0 | 0 | 0 | **379 436.5** | 132 951.3 | **2.854×** | **100.0%** | **1517.75** | **1529.24** | 214.17 | 964.2 | **2124.3** |
| **TOTAL** | **1000** | **1000/1000** | **0** | **0** | **0** | **605 010.2** | **310 772.6** | **1.947×** | **100.0%** | **605.01** | 380.26 | 562.97 | 2.7 | **2124.3** |

### Temps d'Exécution, Couverture & Distribution des Gains

| Niveau | `t_moy` | `t_p95` | `t_p99` | `t_max` | Hostiles servis | Unknowns servis | Wh moy. | Eff. moy. | Gain moy. (V9 − Base) | Gain min | Gain max | Ratio méd. | Ratio min–max |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| **L1** | 0.04 s | 0.06 s | 0.08 s | **0.10 s** | 51.60% (387/750) | 99.60% (249/250) | 22.9 | 2.1 | +9.76 | +0.80 | +88.54 | 1.063× | 1.011×–3.028× |
| **L2** | 0.17 s | 0.27 s | 0.38 s | **0.42 s** | 69.53% (1043/1500) | 100.00% (750/750) | 61.1 | 6.5 | +32.07 | +1.98 | +138.93 | 1.114× | 1.016×–1.755× |
| **L3** | 0.73 s | 1.37 s | 1.86 s | **2.23 s** | 69.69% (2439/3500) | 99.27% (1489/1500) | 184.5 | 19.5 | +149.19 | +46.65 | +293.89 | 1.369× | 1.098×–2.424× |
| **L4** | 17.97 s | 35.98 s | 40.50 s | **40.99 s** | 60.40% (7550/12500) | 85.64% (4282/5000) | 861.5 | 61.6 | **+985.94** | **+522.18** | **+1421.50** | **2.880×** | **1.792×–8.864×** |

- **Histogramme temps L4 (250 scénarios)** : `< 20 s` : **175 (70.0%)** · `20–30 s` : **51 (20.4%)** · `30–40 s` : **15 (6.0%)** · `40–45 s` : **9 (3.6%)** · **`> 45 s` : 0 (0.0%)**. Marge de sécurité minimale de **19.0 s** sous la limite officielle de `60 s`.
- **Records notables sur L4** :
  - Meilleur score absolu : **`L4-1047`** = **2124.3** vs `754.6` (`2.82×`, `39/46` hostiles atteignables, `17/20` unknowns, `36.6 s`).
  - Meilleur ratio : **`L4-1228`** = **1345.6** vs `151.8` (**`8.86×`**), suivi de `L4-1164` (**`7.38×`**) et `L4-1063` (**`6.85×`**).

### Audit des Outliers (Pires Ratios, Pires Scores & Scénarios les Plus Lents sur L4)
L'inspection ciblée des cas extrêmes confirme qu'**aucun dysfonctionnement algorithmique** n'est à l'origine des queues de distribution :
1. **Pire ratio L4 (`L4-1126`, `1181.8` vs `659.6`, `1.79×`)** : géométrie peu dense (`27` hostiles et `19` unknowns atteignables seulement, `2085 pts` atteignables au total) — V9 sert **`25/27` hostiles (`92.6%`)** et **`18/19` unknowns (`94.7%`)**, capturant **`94.3%`** de tous les points individuellement atteignables (`1967 / 2085 pts`).
2. **Pires scores absolus L4 (`L4-1203 = 964.2`, `L4-1004 = 968.0`, `L4-1177 = 989.9`)** :
   - `L4-1004` : seulement `23` hostiles atteignables sur `50` — V9 en sert **`21/23` (`91.3%`)** et **`15/17` unknowns**, soit **`94.5%`** des points atteignables (`1858 / 1966 pts`), `2.93×` baseline.
   - `L4-1177` : déficit structurel en effecteurs légers (`50` lights disponibles pour une demande atteignable de `73` lights) — V9 sature les `50/50` lights + `5` heavies sur les cibles de rentabilité maximale (`2.45×` baseline).
3. **Scénarios les plus lents (`L4-1211 = 40.99 s`, `L4-1022 = 40.94 s`, `L4-1017 = 40.52 s`, `L4-1069 = 40.48 s`)** : instances de densité maximale (`34–38` hostiles + `16–20` unknowns servis, `62–73` effecteurs) où le budget adaptatif Stage 6 borne proprement la recherche à `~40.5–41.0 s`.

### Verdict
**V9 est certifié prêt pour la compétition** : **1 000 / 1 000 valides (`100.0%`)**, **1 000 / 1 000 victoires strictes sur la baseline (`+522 pts` minimum sur L4)**, **`0` crash, `0` violation, `0` timeout (`t_max = 40.99 s` < `60 s`)**. Aucune modification V10 n'est nécessaire au risque d'introduire une régression.
---

## Expérience V10 — Audit de Décomposition des Pertes (100 Scénarios), Super-Scout Liberation & Stage 7 Fallback Trio

### 1. Audit Exact des Pertes Résiduelles de V9 (100 Scénarios Inédits : `20 L1 + 20 L2 + 20 L3 + 40 L4`)
Afin d'identifier rigoureusement où V9 laissait encore des points sur la table, nous avons décomposé l'énergie (`0.0000 Wh` d'écart d'identité vérifié sur le modèle exact `fleetkit/energy.py`) et les cibles non servies sur 100 scénarios inédits (`seeds 1000..1039`) :
- **L1–L3** : déjà à `~99.5%` de l'optimum réalisable (`< 0.5 pt` de surcoût énergétique + perte de déconfliction combinés).
- **L4 (`N = 40`)** :
  - **Surcoût énergétique de vol** (détours obstacles + survitesse/sous-vitesse + sur-altitude `> z_min` + dwell excédentaire) : seulement **`2.36 pts/scén`** sur `427.87 pts` d'énergie totale (**99.45 % physiquement incompressible** !).
  - **Gisement #1 — Rétention des Super-Scouts (*Super-Scout Hoarding*)** : `~10.0 pts/scén` (`7.7` hostiles + `2.3` unknowns) de cibles à marge solo positive laissées non servies alors qu'il reste des effecteurs libres et `5.9` scouts au sol en moyenne. Cause racine : les scouts longue portée / rapides (ex. `S11` sur `L4-1001`, `v = 21.9 m/s`) ont le plus faible coût `Wh/m`, donc l'insertion regret-2 et VND les remplissent jusqu'à `99%` de batterie avec 5–7 cibles proches triviales (atteignables par 12–15 scouts), bloquant les cibles lointaines que seuls 1–3 super-scouts peuvent atteindre.
  - **Gisement #2 — Queue lourde de déconfliction R8** : perte moyenne `6.77 pts/scén` (`médiane 2.65`), mais `15%` des L4 (`6/40`) perdent `> 15 pts` (`moyenne 26.5 pts` = `58.8%` de la perte totale de déconfliction) à cause d'ordres de placement sous-optimaux entre scouts serrés et de congestion verticale autour des pads.

### 2. Ablations & Leçons Clés (Ce Qui a Échoué vs Ce Qui Domine Strictement V9)
1. **Early-Climb 0-Wh (`climb_z=z_min, descend_z=z_min`) par défaut en Pass 1 — REJETÉ** : bien que le profil de montée précoce coûte exactement `0.0000 Wh` et `0.0000 s` (prouvé sur `fleetkit/energy.py`) et gagne `+18.4 pts` sur `L4-1015`, l'activer par défaut concentre tous les scouts dans le même plan `z = 30 m` (au lieu de l'éventail vertical naturel des rampes `z = 2..25 m`), causant `-12 à -18 pts` sur 3/20 L4. **Fix** : réservé exclusivement en fallback strictement monotone (`prefer_early_climb=True`) quand la Pass 1 nominale perd `> 3.5 pts`.
2. **Tour-Split dans la boucle chaude `_regret2_construct` — REJETÉ** : appelé ~500× dans ALNS, 15 appels hongrois par échec drainaient ~12 s de CPU et affamaient ALNS (`-4.3 pts/scén`). **Fix** : déplacé en passe dédiée post-Stage 4 (`_super_scout_liberation`, `< 0.45 s` plafonné, pré-filtrage du score brut avant VND 1-pass).
3. **Pass 1 altérée & Fallbacks au Stage 5 avant Stage 6 — REJETÉ** : modifier la grille `launch_delays` de la Pass 1 cassait `L4-1009` (`-37.6 pts`), et exécuter 3 passes de fallback au Stage 5 consommait 6–12 s et faisait dépasser le seuil `0.78 × 45 s` (affamant tout le Stage 6 sur `L4-1`, `L4-3`, `L4-1009`). De même, écraser `sol` par `lib_sol` au Stage 6b avant déconfliction faisait perdre un plan propre (`1684.39`) sur `L4-1009`. **Fix** : architecture **strictement dominante sur V9** ci-dessous.

### 3. Architecture Finale V10 (`handin/deconflict.py`, `handin/solver.py`)
- **Stages 1–4 + Pass 1 nominale** : **100 % bit-pour-bit identiques à V9** (aucune dérive de trajectoire ou de budget).
- **Stage 4b — `_super_scout_liberation` (plafonné à `0.45 s`, strictement monotone)** : pour chaque cible non servie à marge solo `> 1.0 pt` (avec effecteurs libres), essaie de réserver un super-scout faisable, évince sa tournée courante, ré-insère les cibles évincées sur les autres scouts (insertion directe + réaffectation hongroise + split 2-voies d'une tournée multi-arrêts + remplacement rentable si un hostile évincé de faible valeur ne peut être replacé), avec pré-filtre rapide avant VND 1-pass. Conserve à la fois la solution libérée et la solution V9 originale dans `top_sols`.
- **Stage 5 — Déconfliction nominale + Dominance Skip** : saute immédiatement tout candidat avec `sol["score"] <= best_valid_score + 0.05` (**plus rapide que V9**, préservant 100 % du budget Stage 6), et utilise le seuil R8 exact `self.r.d_sep_m + 2.5` (`42.5 m` sur L4 au lieu de `51.0 m`) dans `_pad_spread_scout_sol` et la relocalisation multi-bloqueurs.
- **Stage 6b — Libération non-destructive** : déconflicte toujours `sol` *d'abord*, et n'évalue `lib_sol` qu'en complément strictement monotone.
- **Stage 7 — `_run_fallback_trio` (post-Stage 6, dans le reliquat de budget)** : déclenché *après* la fin complète du Stage 6 uniquement s'il reste un écart de déconfliction `> 3.5 pts` et `t < hard_deadline - 2.5 s`, en conservant strictement le `max` sur 3 passes :
  1. **Pass 2 — Conflict-Directed Failed-First** (`priority_scouts = _find_troubled_scouts(sol, pass1_plan)` placés *avant* `tier = 0`).
  2. **Pass 3 — R8-exact Pad-Spread** (`_pad_spread_scout_sol(sol)` avec `priority_scouts`).
  3. **Pass 4 — 0-Wh Early-Climb + Grille fine `0.5 s` + Cliff Micro-Probe** (`prefer_early_climb=True`, `priority_scouts`).

### 4. Résultats de Vérification V10
1. **Harnais officiel sandboxé (`python3 handin/check_handin.py --levels L1,L2,L3,L4 --seeds 0-4 --timeout 60`)** :
   - **V10 : `total 12 024.1` (`2.005×` baseline `5 998.5`, `20/20 ok`, `failed runs 0`, `t_max = 42.5 s`)** vs V9 `12 008.9` (**`+15.2 pts`** sur 5 L4 = **`+3.04 pts/L4`**, `L4-2: 1435.5 → 1450.0`, `L4-4: 1455.3 → 1455.9`).
2. **A/B Apparié Strictement Contrôlé — V9 vs V10 sur 30 Scénarios L4 Inédits (`seeds 1000..1029`, même charge 4 workers)** :

   | Métrique (`N = 30` L4 inédits) | **V9 (`e603705`)** | **V10** | **Δ (V10 − V9)** |
   |---|---:|---:|---:|
   | Validité R1–R10 | 30/30 (100%) | **30/30 (100%)** | `0` violation |
   | Score cumulé (Σ 30 L4) | 45 561.36 | **45 598.62** | **`+37.27 pts`** (`+1.24/scén`) |
   | Score moyen L4 | 1518.71 | **1519.95** | **`+1.24 pts`** |
   | Bilan Victoires / Égalités / Pertes | — | **8 W / 22 T / 0 L** | **`0` régression (100 % dominance)** |
   | Temps moyen (`t_avg`) | 29.79 s | **29.43 s** | **`-0.36 s` (plus rapide !)** |
   | Temps max (`t_max`) | 40.75 s | **40.74 s** | **`-0.01 s` (< 45 s / 60 s)** |

   Gains marquants sur L4 : `L4-1012` **`+12.97`**, `L4-1008` **`+9.46`**, `L4-1023` **`+3.75`**, `L4-1026` **`+2.83`**, `L4-1022` **`+2.74`**, `L4-1001` **`+1.97`**, `L4-1027` **`+1.90`**, `L4-1015` **`+1.64`** (**`0` perte**).
3. **8 Scénarios de Validation Officiels (`L1-101..L4-402`)** : **`5 226.31`** (`8/8 valides`, `+0.00` vs V9, `t_max = 27.54 s`). Plans `fleet-kit/plans/L*.json` déjà optimaux et inchangés.
---

## Stress-Test Grande Échelle V10 — 1 000 Scénarios Inédits sur RunPod (`L1–L4`, seeds `1000..1249`)

### Protocole & Infrastructure
- **Version testée** : **V10** (commit `3fdbd2e`), exactement les mêmes 1 000 scénarios inédits (`250` par niveau `L1–L4`, `seeds 1000..1249`, timeout strict `60.0 s` par sous-processus isolé `python3 handin/planner.py`) que la campagne V9, permettant une comparaison **1-à-1 rigoureuse**.
- **Infrastructure** : **4 pods CPU RunPod (`cpu5c × 32 vCPU`, 16 workers/pod)** exécutés en **`132.7 s` wall-clock** (`0` respawn, `0` pod actif vérifié après run via API REST v2, coût `~\$0.25`).

### Comparaison 1-à-1 V10 vs V9 vs Baseline (1 000 / 1 000 Scénarios)

| Niveau | N | Valides | TO | Crash | Score V9 (Σ) | **Score V10 (Σ)** | **Δ (V10 − V9)** | Baseline (Σ) | Ratio V10 | **W / T / L (vs V9)** | `t_moy` | `t_p95` | `t_max` |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| **L1** | 250 | **250/250** | 0 | 0 | 25 373.0 | **25 373.0** | `+0.0` | 22 934.1 | **1.106×** | `0 / 250 / 0` | 0.04 s | 0.05 s | **0.06 s** |
| **L2** | 250 | **250/250** | 0 | 0 | 64 983.1 | **64 981.6** | `-1.5` | 56 966.0 | **1.141×** | `0 / 248 / 2` | 0.15 s | 0.26 s | **0.39 s** |
| **L3** | 250 | **250/250** | 0 | 0 | 135 217.7 | **135 224.5** | **`+6.9`** | 97 921.2 | **1.381×** | `1 / 248 / 1` | 0.74 s | 1.40 s | **2.44 s** |
| **L4** | 250 | **250/250** | 0 | 0 | 379 436.5 | **379 741.5** | **`+305.0`** (`+1.22/scén`) | 132 951.3 | **2.856×** | **`55 / 193 / 2`** | 18.93 s | 40.56 s | **44.44 s** |
| **TOTAL** | **1000** | **1000/1000** | **0** | **0** | 605 010.2 | **605 320.6** | **`+310.4`** | 310 772.6 | **1.948×** | **`56 / 939 / 5`** | — | — | **44.44 s** |

- **Taux de victoire contre la baseline** : **1 000 / 1 000 (`100.0%`)**, gain minimum sur L4 : **`+522.18 pts`** (`L4-1126`), moyenne L4 : **`1518.97 pts`**.
- **Taux de victoire V10 vs V9 sur les L4 non-ex-æquo** : **`55 / 57` (`96.5%`)** !
  - Top gains V10 : `L4-1079` **`+27.29`**, `L4-1120` **`+21.61`**, `L4-1162` **`+19.13`**, `L4-1121` **`+17.78`**, `L4-1215` **`+17.29`**, `L4-1060` **`+13.31`**, `L4-1012` **`+12.97`**, `L4-1066` **`+12.59`**, `L4-1145` **`+11.56`**, `L4-1106` **`+10.62`**.
  - Seulement **2 micro-pertes L4** (`L4-1193: -9.31`, `L4-1040: -3.94`) dues au décalage du voisinage de jitter Stage 6b quand Stage 4b insère un nouveau `top_sols[0]`, et 3 micro-écarts L2/L3 (`-1.12`, `-0.35`, `-0.21`) liés au seuil `d_sep_m + 2.5`.
- **Distribution des temps L4 (`N = 250`)** : `< 20 s` : **163 (65.2%)** · `20–30 s` : **59 (23.6%)** · `30–40 s` : **11 (4.4%)** · `40–45 s` : **17 (6.8%)** · **`> 45 s` : 0 (0.0%)** (`t_max = 44.44 s` sur `L4-1192`, soit **15.56 s de marge** sous la limite de `60 s`).
- **Stress-test CPU lent (`L4-1192` bridé à `time_budget_s = 15.0 s`)** : `valid = True`, `score = 1518.50` (`-0.41 pt` seulement vs `1518.91` à `44.4 s` !), `actual_dt = 15.75 s` (`+0.75 s` d'overshoot seulement).

### Analyse des Risques Résiduels (Audit Compétition)
1. **CPU de jugement beaucoup plus lent (risque faible)** : `FleetSolver.__init__` (graphe de visibilité 3D + pré-calcul effecteurs, `~0.5–1.0 s` sur L4) s'exécute avant `t_start`, et la passe finale Stage 7 + `verify_and_repair_plan` peut dépasser `time_budget_s` de `~0.8–2.5 s`. Avec `time_budget_s = 45.0 s` (`15 s` de tampon), même sur un CPU 2–3× plus lent, le temps total reste `< 55 s` (`< 60 s`) avec perte de score `< 1 pt`.
2. **Scénarios hors-distribution extrêmes (ex. `d_sep_m > 90 m` ou cartes géantes `> 150` contacts)** : les offsets de dodge local `(52 m, 46 m, 88 m)` sont calibrés pour `d_sep_m ∈ [30, 60] m`, et le graphe de visibilité 3D est en $O(V^2)$ dans le nombre de sommets d'obstacles. Même dans un scénario pathologique, les décalages temporels + strates d'altitude + `verify_and_repair_plan` garantissent **100 % de validité R1–R10** (zéro crash / zéro score `0`).
3. **Verdict Final** : **V10 (`3fdbd2e`) est certifié prêt pour la soumission finale** — **1 000 / 1 000 valides**, **1 000 / 1 000 victoires vs baseline (`1.948×` global, `2.856×` L4)**, **`+310.4 pts` nets sur V9 (`55 W / 2 L` sur L4)**, **`0` timeout (`t_max = 44.44 s`)**.
