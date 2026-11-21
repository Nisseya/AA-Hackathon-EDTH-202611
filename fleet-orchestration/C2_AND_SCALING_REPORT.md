# Orchestration C2 Explicable, Regroupement d'Approbations Atomiques et Étude de Passage à l'Échelle (5 à 500 Drones)

> **Contexte** : Rapport de recherche compagnon au planner **V10** ([`RESEARCH_LOG.md`](RESEARCH_LOG.md), certifié **1 000 / 1 000 valides**, `1.948×` baseline global et `2.856×` L4 sur RunPod).  
> **Objectif** : Couvrir de manière rigoureuse, mesurable et reproductible les composantes C2 & passage à l'échelle de la fiche track *Fleet Orchestration (`SUB-15`, Alta Ares — EDTH Paris 2026)* sans modifier le dossier certifié [`fleet-kit/handin/`](fleet-kit/handin/) :
> - **Couche B — Explicabilité opérateur** ([`c2_companion/explain.py`](c2_companion/explain.py)) : justification déterministe de chaque décision d'engagement *et* audit contrefactuel de chaque contact non servi.
> - **Couche C — Réduction d'approbations (*Fewer Approvals*)** ([`c2_companion/packages.py`](c2_companion/packages.py)) : regroupement en *Strike Packages* atomiques à **veto garanti sans cascade sous R1–R10**, stratifiés par risque ROE, et intégration dans le Viewer HTML offline ([`c2_companion/demo_viewer.py`](c2_companion/demo_viewer.py)).
> - **Couche D — Étude de passage à l'échelle (*Scaling Study*, 5 → 500 drones)** ([`c2_companion/scaling_bench.py`](c2_companion/scaling_bench.py)) : profilage par étage, lois de puissance empiriques $T(N) \sim c N^\alpha$, point de rupture exact et prototype de mitigation algorithmique.
> - **Annexe E — Audit des règles (*« What breaks the rules? »*)** : analyse formelle et preuves exécutables des subtilités et *loopholes* de [`validator.py`](fleet-kit/fleetkit/validator.py).

---

## 1. Expérience 1 — Couche B : Moteur d'Explicabilité Déterministe et Audit Contrefactuel

### 1.1 Intuition & Formulation
Dans un système C2 militaire (*Command & Control*), un opérateur humain ne peut pas valider un plan JSON opaque contenant jusqu'à 100 trajectoires 3D. Une explicabilité exploitable doit répondre à **trois questions complémentaires** :
1. **Pourquoi ce contact est-il servi ?** Chaîne de frappe complète (*kill-chain* : scout confirmateur, marge temporelle **R4**, synchronisation **R10**, marge de fenêtre **R9**, distance collatérale **R7**, séparation 3D **R8**), décomposition **additive exacte** du coût (effecteurs + part proportionnelle d'énergie du scout confirmateur) donnant un **ROI tactique net**, et classification en **4 niveaux de criticité ROE** :
   - `LOW_RECON` : contact `unknown`, ISR non létal uniquement ;
   - `STANDARD_STRIKE` : frappe `hostile` avec marges confortables sur toutes les règles ;
   - `TIGHT_MARGIN_STRIKE` : frappe `hostile` avec marge R4 `< 5 s`, R9 `< 10 s` ou R8 `< 50 m` ;
   - `HIGH_SCRUTINY_COLLATERAL` : frappe `hostile` à moins de `400 m` d'un contact ami/civil (dans la bande tampon `[200 m, 400 m]` au-dessus du rayon collatéral interdit R7).
2. **Pourquoi ce contact n'est-il PAS servi ? (Audit contrefactuel hiérarchique)** : au lieu de simplement marquer un contact comme « ignoré », le moteur évalue les bornes physiques exactes (énergie minimale à vitesse `best_range_speed`, temps cinématique minimal, rayon collatéral R7, effecteurs résiduels) pour attribuer une **cause racine déterministe unique** :
   - `R7_COLLATERAL_VETO` : légalement inengageable (`d < 200 m` d'un ami/civil) ;
   - `ENERGY_RANGE_INFEASIBLE` : hors rayon batterie (R3) même isolément ;
   - `WINDOW_R9_INFEASIBLE` : impossible à atteindre dans `[t_lo, t_hi]` (R9) même à vitesse max ;
   - `NEGATIVE_ISOLATED_ROI` : `points <= coût_minimal_isolé` (structurellement déficitaire) ;
   - `FLEET_EFFECTOR_SATURATED` : effecteurs consommés sur des cibles à meilleur rendement (`pts/effecteur`) ;
   - `SCOUT_OR_R8_BOTTLENECK` : faisable isolément avec effecteurs libres, mais bloqué par la saturation des scouts ou la congestion R8 — mesure directe de la **marge résiduelle d'optimisation**.
3. **Quelle est l'enveloppe de sécurité globale de la flotte ?** Calcul exact de la plus faible séparation 3D airborne (R8, avec exemption cible `exempt_radius_m = 100 m`), des marges minimales NFZ/zones protégées (R6) et des réserves batterie (R3).

### 1.2 Protocole Expérimental
- **Script reproductible** : [`c2_companion/exp1_explain.py`](c2_companion/exp1_explain.py) appelant [`explain_plan()`](c2_companion/explain.py#L186).
- **Jeu de test** : **28 scénarios** couvrant les 4 niveaux — les **8 scénarios officiels de validation** (`L1-101..L4-402`, plans V10 + comparaison Baseline Gloutonne) + **20 scénarios `check_handin` non vus** (`L1..L4`, seeds `0..4`, résolus avec V10 à budget `45 s`).
- **Invariants vérifiés** : validité R1–R10 (`28/28`), erreur maximale de conservation additive des coûts $|\text{Coût}_{\text{exact}} - (\text{Coût}_{\text{attribué}} + \text{Coût}_{\text{scouts sans cible}})|$, et latence d'explication.

### 1.3 Résultats Quantitatifs

| Niveau | $n$ | Servis | Non servis | Capture légale (%) | R8 min (m) | R4 min (s) | R10 max (s) | Réserve batt. min (S / E) | Latence `explain` moy. (max) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **L1** | 6 | 15 | 9 | 63.6% | 42.64 | +0.05 | 6.00 | 47.6% / 83.3% | 1.7 ms (3.7 ms) |
| **L2** | 7 | 52 | 11 | 86.6% | 44.62 | +0.05 | 17.73 | 14.9% / 81.7% | 3.0 ms (5.4 ms) |
| **L3** | 8 | 129 | 31 | 92.2% | 40.48 | +0.05 | 19.85 | 2.3% / 68.4% | 11.7 ms (19.3 ms) |
| **L4** | 7 | 336 | 154 | 81.1% | 40.36 | +0.05 | 19.85 | 0.1% / 56.0% | 93.5 ms (129.4 ms) |

- **Conservation additive exacte** : erreur maximale = **`0.03 pts`** (arrondi à 2 décimales uniquement) sur les 28 plans (`28/28` valides).
- **Latence temps réel** : `< 0.13 s` même pour un théâtre L4 complet (100 drones, 90 contacts).

**Décomposition contrefactuelle globale des 205 contacts non servis (12 654 pts bruts sur 28 scénarios)** :

| Cause racine déterministe | Contacts | Part contacts | Points bruts | Part points | Interprétation C2 |
| :--- | :---: | :---: | :---: | :---: | :--- |
| `R7_COLLATERAL_VETO` | 105 | 51.2% | 7 122.0 | **56.3%** | Légalement interdit (ami/civil `< 200 m`) |
| `WINDOW_R9_INFEASIBLE` | 31 | 15.1% | 1 848.0 | **14.6%** | Cinématiquement hors fenêtre temporelle R9 |
| `FLEET_EFFECTOR_SATURATED` | 25 | 12.2% | 1 604.0 | **12.7%** | Arbitrage économique (effecteurs mieux investis) |
| `NEGATIVE_ISOLATED_ROI` | 35 | 17.1% | 1 587.0 | **12.5%** | Déficitaire même isolément (`pts <= coût min`) |
| `SCOUT_OR_R8_BOTTLENECK` | 9 | 4.4% | 493.0 | **3.9%** | Seul goulot d'étranglement résiduel (scouts / R8) |

**Comparaison explicable V10 vs Baseline sur les 8 scénarios de validation** :

| Scénario | Score V10 | Capture légale V10 | Score Baseline | Capture légale Base | Causes des contacts non servis (V10) |
| :--- | :---: | :---: | :---: | :---: | :--- |
| `L1-101` | **72.5** | 50.7% (2) | 68.0 | 50.7% (2) | 2× `FLEET_EFFECTOR_SATURATED` |
| `L2-201` | **299.9** | 88.7% (7) | 277.8 | 88.7% (7) | 1× `R7_VETO`, 1× `EFF_SAT` |
| `L2-202` | **267.3** | **100.0%** (7) | 232.9 | 96.1% (6) | 2× `R7_VETO` (0 perte) |
| `L3-301` | **729.1** | **100.0%** (18) | 423.8 | 66.9% (10) | 2× `R7_VETO` (0 perte) |
| `L3-302` | **423.0** | 83.8% (14) | 326.5 | 79.1% (12) | 3× `R7_VETO`, 2× `EFF_SAT`, 1× `NEG_ROI` |
| `L3-303` | **455.8** | 82.7% (16) | 275.9 | 58.8% (11) | 2× `NEG_ROI`, 1× `EFF_SAT`, 1× `R7_VETO` |
| `L4-401` | **1 717.7** | **85.5%** (52) | 641.1 | 33.5% (16) | 9× `R7_VETO`, 5× `R9_INFEAS`, 4× `NEG_ROI` (**0 bottleneck**) |
| `L4-402` | **1 261.0** | **80.7%** (47) | 418.5 | 29.7% (14) | 12× `R7_VETO`, 6× `R9_INFEAS`, 5× `NEG_ROI` (**0 bottleneck**) |

### 1.4 Enseignements Clés (Expérience 1)
1. **Certificat de quasi-optimalité** : Sur les 28 scénarios, **96.1% des points non servis** s'expliquent par des impossibilités physiques/légales ou un arbitrage économique rationnel (R7, R9, ROI négatif, saturation d'effecteurs). Sur `L4-401` et `L4-402`, **100% des contacts ignorés** par V10 sont prouvés inengageables, hors fenêtre ou déficitaires (`0` contact perdu par goulot scout/R8) — alors que la baseline laisse ~67–70% de la valeur légale sur la table.
2. **Profil de criticité ROE** : Sur les 532 contacts servis, `37.6%` sont de l'ISR non létal (`LOW_RECON`), **`9.2%` (49 contacts)** sont des frappes proches de civils/amis (`HIGH_SCRUTINY_COLLATERAL`, `200–400 m`) qui méritent une revue humaine isolée, et `50.4%` sont en `TIGHT_MARGIN_STRIKE` principalement car V10 serre délibérément la marge R4 à `+0.05 s` pour compacter les tournées scouts. Ce constat motive directement la **stratification ROE** de l'Expérience 2.

---

## 2. Expérience 2 — Couche C : Réduction d'Approbations (*Fewer Approvals*) & Couches Viewer C2

### 2.1 Intuition, Théorème de Veto Sûr & Stratification ROE
Sur un théâtre L4, le planner V10 active en moyenne **69.9 drones** servant **48.0 contacts**. Demander à un commandant C2 d'approuver chaque drone ou chaque cible individuellement crée une surcharge cognitive critique (*approval fatigue*).  
En revanche, un regroupement géométrique naïf **casse la validité R1–R10 en cas de veto** : si deux cibles partagent le même scout confirmateur et que l'opérateur rejette l'une des deux, la suppression naïve du scout invalide l'engagement de l'autre (violation **R4**).

Nous introduisons donc une architecture en deux étages ([`c2_companion/packages.py`](c2_companion/packages.py)) :
1. **Décomposition en Composantes Connexes Atomiques (`ATOMIC_CC`)** : construction du graphe biparti de dépendances entre appareils actifs et contacts servis (arêtes `(scout, contact)` et `(effecteur, contact)`). Chaque composante connexe est une unité indivisible.
2. **Regroupement Spatio-Temporel Stratifié par Risque ROE (`ROE_WAVE_K`)** : fusion agglomérative des composantes atomiques *strictement à l'intérieur de 3 strates ROE hermétiques* (on ne mélange jamais ISR non létal et frappe proche de civils) :
   - `RECON_ONLY` : ISR pur (`unknown`, 0 effecteur) → 1 approbation groupée ;
   - `STANDARD_STRIKE` : frappes `hostile` avec marge collatérale R7 $\ge 400\text{ m}$ → fusionnées par proximité espace-temps avec régularisation *Value-at-Risk* (évitant qu'un seul paquet n'absorbe trop de valeur) ;
   - `HIGH_SCRUTINY_STRIKE` : frappes `hostile` à marge R7 dans $[200\text{ m}, 400\text{ m})$ → isolées dans des paquets dédiés à haute vigilance.

> **Théorème (Veto Sûr Sans Cascade sous R1–R10)** : Si un plan est valide et que chaque *Strike Package* $P_k$ est une union disjointe de composantes connexes de dépendances, alors **rejeter n'importe quel sous-ensemble arbitraire $\mathcal{R} \subseteq \{P_1, \dots, P_K\}$** en supprimant simplement les routes des appareils de $\mathcal{R}$ produit **nécessairement un plan 100% valide sous R1–R10** (aucune nouvelle violation R4, R10 ou R8 possible : retirer des drones ne peut qu'enlever des paires de séparation R8).

### 2.2 Protocole Expérimental
- **Script reproductible** : [`c2_companion/exp2_packages.py`](c2_companion/exp2_packages.py).
- **Jeu de test** : les mêmes **28 scénarios** (8 validation + 20 `check_handin`), en comparant 4 politiques : `ATOMIC_CC`, `ROE_WAVE_K` avec $K_{\max} \in \{8, 6, 4\}$.
- **Certification empirique du Veto Sûr** : exécution réelle du validateur officiel [`fleetkit.scorer.evaluate`](fleet-kit/fleetkit/scorer.py) après suppression de **chaque paquet individuellement** (*Single-Veto*) ET de **8 sous-ensembles aléatoires multi-paquets** (*Multi-Veto*) par scénario et par politique — soit **910 tests de veto complets**.
- **Intégration Viewer C2** : génération de pages HTML autonomes offline via [`c2_companion/demo_viewer.py`](c2_companion/demo_viewer.py) pour `L3-301` et `L4-401` avec enveloppes convexes des paquets, liens *kill-chain* R4 temporisés, marqueurs de contacts rejetés (audit contrefactuel) et point de séparation R8 minimal.

### 2.3 Résultats Quantitatifs

| Politique | Niveau | Routes actives | Cibles servies | CCs atomiques | **Paquets ($K$)** | **Réduction vs Routes** | **Réduction vs Cibles** | Max VaR / paquet | **Certification Veto Sûr** |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| `ATOMIC_CC` | L1 | 3.2 | 2.5 | 1.0 | **1.0** | -65.3% | -58.4% | 100.0% | 6/6 (100%) |
| `ATOMIC_CC` | L2 | 8.7 | 7.4 | 1.4 | **1.4** | -83.7% | -81.1% | 87.7% | 7/7 (100%) |
| `ATOMIC_CC` | L3 | 22.1 | 16.1 | 2.5 | **2.5** | -88.7% | -84.4% | 58.2% | 8/8 (100%) |
| `ATOMIC_CC` | **L4** | 69.9 | 48.0 | 9.3 | **9.3** | **-86.7%** | **-80.6%** | **22.3%** | **7/7 (100%)** |
| `ROE_WAVE_K=8` | **L4** | 69.9 | 48.0 | 9.3 | **7.9** | **-88.7%** | **-83.6%** | **23.5%** | **7/7 (100%)** |
| `ROE_WAVE_K=6` | **L4** | 69.9 | 48.0 | 9.3 | **6.0** | **-91.4%** | **-87.4%** | **33.5%** | **7/7 (100%)** |
| `ROE_WAVE_K=4` | **L4** | 69.9 | 48.0 | 9.3 | **4.0** | **-94.2%** | **-91.6%** | **47.3%** | **7/7 (100%)** |

- **Certification Veto Sûr Globale** : **334 / 334** vetos simples valides + **576 / 576** vetos multi-paquets aléatoires valides = **910 / 910 (100.0% valides, 0 violation R1–R10)**.
- **Front de Pareto Charge Cognitive vs Granularité du Veto (L4)** : `ROE_WAVE_K=6` constitue le point d'équilibre optimal : la charge opérateur chute de **69.9 routes → 6.0 paquets (`-91.4%`)**, tout en maintenant la valeur à risque maximale d'un veto à `33.5%` et en isolant systématiquement les frappes proches de civils (`HIGH_SCRUTINY_STRIKE`) des frappes propres (`STANDARD_STRIKE`) et de la reconnaissance (`RECON_ONLY`).

**Illustration concrète — Démo Jury `L4-401` ([`demos/c2_demo_L4-401.html`](demos/c2_demo_L4-401.html), 66 drones → 6 approbations, `-90.9%`)** :
- `PKG-01 [HIGH_SCRUTINY_STRIKE]` — CENTER, min R7 `255 m`, `145–521 s` : 7 cibles (1S + 11E), net `+256.6 pts` (14.9%)
- `PKG-02 [STANDARD_STRIKE]` — SE, `161–743 s` : 20 cibles (2S + 28E), net `+775.7 pts` (45.2%)
- `PKG-03 [RECON_ONLY]` — ISR Sweep CENTER, `218–566 s` : 8 cibles `unknown` (2S + 0E), net `+159.0 pts` (9.3%)
- `PKG-04 [HIGH_SCRUTINY_STRIKE]` — NW, min R7 `218 m`, `254–582 s` : 7 cibles (1S + 11E), net `+241.8 pts` (14.1%)
- `PKG-05 [STANDARD_STRIKE]` — SE, `262–608 s` : 6 cibles (1S + 7E), net `+189.0 pts` (11.0%)
- `PKG-06 [STANDARD_STRIKE]` — NW, `377–721 s` : 4 cibles (1S + 1E), net `+95.7 pts` (5.6%)

*(Bug détecté et corrigé durant l'expérimentation : la première passe incluait les contacts `unknown` dans le calcul de `min_collateral_m` d'un paquet mixte, affichant `125 m` alors que R7 ne s'applique qu'aux frappes létales `hostile`. Le filtrage strict sur `HOSTILE` dans [`packages.py`](c2_companion/packages.py) a corrigé la métrique à `255 m` et `218 m`, strictement $\ge 200\text{ m}$.)*

---

## 3. Expérience 3 — Couche D (Partie I) : Étude de Passage à l'Échelle de V10 (5 → 400 Drones) & Point de Rupture

### 3.1 Question de Recherche & Protocole
La fiche track demande explicitement : *« Scaling study: how far can your planner go before it breaks (30 to 100+ aircraft)? »*  
Notre planner certifié **V10** est validé sur **1 000 scénarios officiels L1–L4** (RunPod, seeds `1000..1249`, budget `45 s`) avec les percentiles de latence suivants :

| Niveau ($N$ drones) | $n$ | Score V10 | Baseline | Ratio | $t_{\text{moy}}$ | $p_{50}$ | $p_{90}$ | $p_{99}$ | $t_{\max}$ | Valides |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **L1** (`5`) | 250 | 101.5 | 91.7 | `1.106×` | 0.04 s | 0.04 s | 0.05 s | 0.06 s | 0.06 s | 250/250 |
| **L2** (`12`) | 250 | 259.9 | 227.9 | `1.141×` | 0.15 s | 0.14 s | 0.23 s | 0.30 s | 0.39 s | 250/250 |
| **L3** (`30`) | 250 | 540.9 | 391.7 | `1.381×` | 0.74 s | 0.66 s | 1.17 s | 2.04 s | 2.44 s | 250/250 |
| **L4** (`100`) | 250 | 1 519.0 | 531.8 | **`2.856×`** | 18.93 s | 17.18 s | 31.65 s | 42.98 s | 44.44 s | 250/250 |

Pour identifier **où et pourquoi** V10 casse au-delà de L4, nous avons construit [`c2_companion/scaling_bench.py`](c2_companion/scaling_bench.py) :
- **Générateur étendu physiquement cohérent** (`N ∈ {5, 12, 30, 60, 100, 150, 200, 225, 250, 300, 400}`, proportions exactes de L4 : `15%` scouts, `50%` light, `35%` heavy, `0.90 N` contacts) sous **deux régimes** :
  1. *Régime A (`fixed_theater`)* : théâtre fixe `7 000 m`, `8` zones — teste la densification pure (congestion pad R8, collatéraux R7, combinatoire).
  2. *Régime B (`scaled_area`)* : aire $\propto N^{0.25}$ (capée à `9 000 m` pour rester sous l'horizon batterie) et zones $\propto N^{0.5}$.
- **Instrumentation non invasive** via la sous-classe [`InstrumentedFleetSolver`](c2_companion/scaling_bench.py#L90) mesurant `t_init`, `t_knapsack`, `t_eval_subset`, `t_deconflict` et `t_validate` (sans modifier `handin/`).

### 3.2 Résultats — Courbe de Passage à l'Échelle (Régime A : `fixed_theater`)

| $N$ drones | Contacts | Valides | $\le 60\text{ s}$ | $t_{\text{tot}}$ | `t_init` | `knap/c` | `eval/c` (×appels) | `deconf/c` (×appels) | `validate` | Score V10 | vs Base | Cibles servies |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **5** (L1) | 5 | 3/3 | 3/3 | 0.01 s | 0.00 s | 0.000 s | 0.000 s (×4.0) | 0.00 s (×1.0) | 0.0002 s | 66.3 | `1.14×` | 2.0 |
| **12** (L2) | 11 | 3/3 | 3/3 | 0.25 s | 0.01 s | 0.000 s | 0.017 s (×13.3) | 0.01 s (×1.0) | 0.0009 s | 254.6 | `1.23×` | 7.0 |
| **30** (L3) | 25 | 3/3 | 3/3 | 2.20 s | 0.03 s | 0.0003 s | 0.561 s (×4.7) | 0.03 s (×1.0) | 0.0029 s | 632.9 | `1.42×` | 16.7 |
| **60** | 54 | 3/3 | 3/3 | 6.24 s | 0.27 s | 0.0017 s | 0.585 s (×11.7) | 0.17 s (×2.7) | 0.0183 s | 971.6 | `2.71×` | 29.3 |
| **100** (L4) | 90 | 3/3 | 3/3 | 25.88 s | 1.28 s | 0.0063 s | 1.974 s (×17.0) | 0.51 s (×3.0) | 0.0388 s | 1 414.4 | `3.37×` | 44.7 |
| **150** | 135 | 3/3 | 3/3 | 43.59 s | 3.23 s | 0.0214 s | 11.446 s (×3.3) | 1.77 s (×3.3) | 0.1040 s | 2 232.0 | `3.37×` | 68.7 |
| **200** | 180 | 3/3 | 3/3 | 52.76 s | 7.04 s | 0.0567 s | 27.063 s (×1.3) | 5.46 s (×2.7) | 0.2488 s | 3 009.7 | **`4.38×`** | 94.7 |
| **225** | 202 | 2/2 | 2/2 | 53.40 s | 9.06 s | 0.0549 s | 32.611 s (×1.0) | 4.55 s (×2.5) | 0.1784 s | 3 308.9 | — | 110.5 |
| **250** | 225 | 2/2 | **2/2** *(max 59.2 s)* | 54.00 s | 7.24 s | 0.0706 s | 32.602 s (×1.0) | 10.06 s (×2.0) | 0.1620 s | 3 598.7 | — | 114.0 |
| **300** | 270 | 3/3 | **0/3** | **74.47 s** | **26.70 s** | 0.1545 s | 32.894 s (×1.0) | **13.96 s** (×1.0) | 0.2491 s | **4 017.5** | — | **134.0** |
| **400** | 360 | 2/2 | **0/2** | **102.07 s** | **49.60 s** | 0.2399 s | 34.499 s (×1.0) | **15.23 s** (×1.0) | 0.2974 s | **3 372.9** *(chute!)* | — | **94.0** |

*Régime B (`scaled_area`)* : résultats concordants — `N=100` : `40.46 s` (`2.59×` base) ; `N=200` : `49.97 s` (`3.29×` base, `2/2 <= 60 s`) ; `N=300` : `83.07 s` (`0/2 <= 60 s`, `t_init = 23.93 s`, `deconf = 25.88 s`).

### 3.3 Lois de Puissance Empiriques & Diagnostic des Deux Points de Rupture
La régression log-log sur $N \in [30, 300]$ (Régime A) révèle avec $R^2 > 0.994$ la structure asymptotique exacte de chaque sous-système :

| Sous-système | Loi de puissance empirique | $R^2$ | Complexité théorique & cause racine |
| :--- | :--- | :---: | :--- |
| **`t_init` (`FleetSolver.__init__`)** | $T(N) pprox 1.29 	imes 10^{-6} \cdot N^{2.95}$ | **0.9960** | **$O(N^3)$** : boucle `(N_hostile × N_effector)` appelant [`effector_min_cost()`](fleet-kit/handin/timing.py#L629) → `best_path_3d()` contre `O(N)` cylindres protégés (`0.20 N` amis/civils). À $N=400$, `__init__` brûle seul **`49.60 s`** (et n'était pas décompté de `time_budget_s` !). |
| **`t_deconf_per_call`** | $T(N) pprox 2.05 	imes 10^{-6} \cdot N^{2.75}$ | **0.9948** | **$O(N^{2.75})$** : [`SeparationIndex.first_conflict()`](fleet-kit/handin/deconflict.py#L130) scanne linéairement toutes les routes déjà placées sans indexation spatio-temporelle, multiplié par l'augmentation du nombre d'essais R8 sur un pad congestionné. |
| **`t_knap_per_call`** | $T(N) pprox 2.34 	imes 10^{-8} \cdot N^{2.75}$ | **0.9977** | **$O(N_h \cdot N_L \cdot N_H) = O(N^3)$** : DP exacte 2D, mais constante très faible (`0.24 s` même à `N=400`). |
| **`fleetkit.validate()`** | $T(N) pprox 3.72 	imes 10^{-6} \cdot N^{2.02}$ | 0.9699 | **$O(N^2)$** : boucle double `_r8` du validateur officiel (`0.30 s` à `N=400`, non limitant). |

**Deux points de rupture distincts identifiés** :
1. **Point de rupture temps réel ($60\text{ s}$) : $N^*_{60\text{s}} \approx 250\text{ drones}$ (`2.5×` L4)** — V10 reste **100% valide** (`36/36` runs jusqu'à `N=400`) et atteint **`4.38×` baseline à `N=200`**, mais dépasse `60 s` dès `N > 250` car `t_init` ($O(N^{2.95})$, hors budget) + `search_deadline` fixe (`32.4 s`) + `deconflict` ($O(N^{2.75})$) dépassent mécaniquement `60 s`.
2. **Point de rupture qualité / physique : $N \approx 350\text{–}400\text{ drones}$** — Le score passe de `4 018` (`N=300`) à `3 373` (`N=400`) : d'une part, la 1re passe `evaluate_subset` n'arrive plus à converger en `32 s` ; d'autre part, dans un théâtre fixe `7×7 km` à pad unique `100×100 m`, l'espacement entre pads tombe à `5.2 m` (`< d_sep_m = 40 m`) et 80 contacts amis/civils couvrent >20% de la carte en rayons collatéraux R7.

---

## 5. Annexe E — Audit Exécutable des Règles R1–R10 (*« What breaks the rules? »* & *Loopholes*)

La fiche track pose explicitement la question *« What breaks the rules? »* et le [`README.md`](README.md#L47) invite les équipes à signaler les *loopholes* : *« A plan the rules accept can still be silly; tell us if you find a loophole. »*  
Nous avons audité [`fleetkit/validator.py`](fleet-kit/fleetkit/validator.py) et construit **5 preuves de concept exécutables** ([`c2_companion/rule_audit.py`](c2_companion/rule_audit.py)), toutes vérifiées directement contre le validateur officiel (`5/5` acceptées avec **0 violation**) :

| ID | Règle | Sévérité | Accepté par `validate()` | Constat technique & impact |
| :--- | :---: | :---: | :---: | :--- |
| `LOOPHOLE-R2-GROUND-CRAWL` | **R2** | **Haute** | **Oui (0 viol.)** | Un effecteur à 2 waypoints `[home(z=0) → target(z=0)]` vole à **`z = 0.0 m`** sur la **totalité** de son vol (`1 988 m`), n'entre jamais dans `[30 m, 150 m]`, paie `0 Wh` d'énergie de montée (`-0.72 Wh`, `+0.36 pts`/effecteur) et passe `_r2` car le dernier waypoint autorise `low = 0.0` et l'altitude des *segments* n'est jamais vérifiée. |
| `LOOPHOLE-R2-SUB-ZMIN-SKIM` | **R2** | Moyenne | **Oui (0 viol.)** | Un scout à 3 waypoints `(home, 0) → (target, 30 m) → (home, 0)` passe 100% de ses deux trajets de croisière (`1 932 m` chacun) strictement sous `z_min = 30 m` (altitude moyenne `15 m`), accepté car `_r2` ne teste que les waypoints intermédiaires. |
| `LOOPHOLE-R8-MIDAIR-COLLISION` | **R8** | Moyenne | **Oui (0 viol.)** | Deux effecteurs visant le même hostile peuvent **entrer en collision exacte (`0.0 m` en 3D à `z = 35 m`, à `61 m` de la cible)** `8 à 22 s` *avant* l'impact (avec `14 s` d'écart entre frappes), et `_r8` l'accepte car les deux sont dans la sphère `exempt_radius_m = 100 m`. |
| `SPEC-R10-AIRBURST-25M` | **R10** | Basse *(physique)* | **Oui (0 viol.)** | L'engagement vérifie `dist3 <= R_eng_m = 25 m` : un effecteur peut engager en *airburst* à `z = 24.9 m` directement au-dessus de la cible sans piquer jusqu'au sol, économisant du temps de descente. |
| `SPEC-R8-GROUND-CLOAK` | **R8 + Gen.** | Moyenne *(réalisme)* | **Oui (0 viol.)** | En L4 (100 drones), les pads du générateur sont espacés de **`11.1 m`** (`< d_sep_m = 40 m`). Comme `route_pieces()` ignore les appareils avant `w[0].depart`, un drone qui décolle survole ses voisins garés à `11.1 m` sans violation (« cape au sol »), mais deux décollages simultanés à `< 40 m` violent immédiatement R8 — créant un goulot physique de débit de décollage à très grande échelle ($N \ge 200$). |

**Recommandations simples de durcissement pour les organisateurs** :
1. Dans `_r2`, exiger qu'une route d'effecteur atteigne `z >= z_min_m` sur au moins un waypoint de croisière (ou vérifier la pente minimale de montée/descente près de `home`/`target`).
2. Dans `_r8`, restreindre l'exemption `exempt_radius_m` aux `Δt <= 5 s` précédant l'engagement ou à un rayon terminal plus serré (`30–50 m`).
3. Dans le générateur au-delà de L4, étendre l'empreinte de base proportionnellement à $\sqrt{N}$ (ou ajouter plusieurs bases déportées) pour éviter la congestion artificielle d'un pad unique de `100 × 100 m`.

---
