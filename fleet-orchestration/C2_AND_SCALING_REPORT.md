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
