# go2-planner

Unitree Go2 파쿠르에서 **룰베이스 스킬 플래너**와 **E2E 학습 정책**을 같은 조건으로 비교하는
연구용 리포입니다. 실 로봇 텔레옵 로그 36세션에서 이산 스킬(WALK/TROT/RUN/JUMP)을 추출해 개루프
관절 재생으로 되살리고, 룰 엔진이 깊이 카메라가 볼 수 있는 것만 보고 스킬을 고르게 한 뒤,
eurekaverse의 **benchmark 20태스크 × 난이도 10단계**(Isaac Lab)에서 E2E 연속 정책 및
단일 스킬 하한선과 점수를 겨룹니다. 지금 단계에서 리포에 들어 있는 것은 대부분 그 비교를
성립시키기 위한 **계측 결과**입니다 — 지형 기하, 스킬 실측 능력, 전환 비용, 그리고 개루프 재생이
어디까지 되는지.

작업을 시작하기 전에 **[`CLAUDE.md`](CLAUDE.md)를 먼저 읽으세요.** 경로 권한(읽기 전용 트리),
임계값을 성능 데이터로 조정하지 않는다는 규칙, upstream 버그를 고치지 않는다는 규칙,
`curated/` 로그 취급 주의사항이 거기 있습니다. 이 README는 "무엇이 어디 있는가"만 다룹니다.

---

## 디렉터리

| 경로 | 역할 |
|---|---|
| `terrain_toolkit/` | 벤치마크 지형 툴킷 (CPU 전용, Isaac/torch 미임포트). upstream `set_terrain_benchmark.py`를 읽기 전용으로 실행해 20×10 지형을 동결하고(`freeze.py`), goal 구간별 기하 특성을 뽑고(`profile.py`), 임계값 캘리브레이션용 별도 프로브 지형을 만듭니다(`calibrate.py`). 모든 경로 해석은 `paths.py`가 담당 — 하드코딩 금지 |
| `motion_toolkit/` | `curated/` 로그 36세션 분석 (CPU 전용). 세션 로딩(`session.py`), 관절 속도 기반 모션 구간 판정(`window.py`), 세션·다리별 접촉 재산출(`contact.py`), 세션당 ~100개 특성(`profile.py`), 도약 분석(`jump.py`), 전환 비용(`transitions.py`), 재생용 클립 추출(`clips.py`) |
| `planner/` | 룰베이스 이산 스킬 플래너 (CPU 전용). 모든 파라미터에 `MEASURED`/`DERIVED`/`CALIBRATION_NEEDED`/`CONVENTION` 출처 태그가 붙은 `config.py`, 깊이 카메라 시야를 그대로 흉내 내는 관측 모델 `features.py`, 스킬 정의 `skills.py`, 히스테리시스·최소 유지시간·단일 명령 채널을 갖춘 룰 엔진 `rules.py`, 대안 점프 게이트 `tracking.py` |
| `sim/` | Isaac 쪽 헬퍼. 클립 재생 4가지 모드(`replay.py`), 관절 순서·부호·게인 오진단 도구(`diagnose.py`), upstream 설정을 `ast`로 파싱해 읽는 `isaac_cfg.py`, 하이트필드→메시 변환(`heightfield.py`). `diagnose.py` 외에는 Isaac Lab이 있어야 임포트됩니다 |
| `scripts/` | 실행 진입점 전부. 동결(`freeze_*`), 프로파일링(`profile_*`), 클립 추출과 재추출 게이트(`extract_skill_clips.py`, `gate_reextract.py`), 오프라인 플래너 스윕(`simulate_planner_offline.py`), Isaac 재생 검증(`verify_skill_replay.py`), 분석(`analyze_drift.py`, `analyze_heading_budget.py`), 컨테이너 러너(`isaac_docker_run.sh`) |
| `data/` | 동결 아카이브와 해시. `benchmark_frozen.npz`(정본 평가 지형), `skill_clips.npz`(재생 클립 4개, sha256 `aab85a03…`), `calibration_probes.npz`, `raw_cycles_TROT.npz`, `upstream_commit.txt`. **재생성 금지** — 해시로 검증합니다 |
| `outputs/` | 모든 분석 산출물 (아래 절 참조). `.gitignore`가 `outputs/*`를 무시하므로 커밋 대상은 개별 force-add 합니다 |
| `logs/` | 실행 콘솔 로그와 원시 트레이스 npz. GitHub에 없습니다 |
| `eval/`, `skills/` | 아직 비어 있음 (플레이스홀더) |

**동결 지형에 대한 주의.** `data/benchmark_frozen.npz`는 upstream 대비 두 가지가 **미적용**입니다:
`random_uniform_terrain` 노이즈(±0.02–0.06 m)와 셀 테두리 0.5 m 패드. 시뮬레이터는 양쪽 실험군에
똑같이 적용하므로 비교는 공정하지만, 아카이브에서 뽑은 **거칠기(roughness) 수치를 시뮬 표면의
성질처럼 인용하면 안 됩니다.** 또 정본 배열은 `height_fields_before_fix` 입니다 — 동결 커밋에서
upstream의 benchmark 분기는 `fix_terrain`을 호출하지 않으므로, 그쪽이 시뮬이 실제로 래스터화하는
배열입니다. 자세한 내용은 [`terrain_toolkit/README.md`](terrain_toolkit/README.md).

각 툴킷의 설계 근거와 알려진 한계는 하위 README에 있습니다:
[`terrain_toolkit/README.md`](terrain_toolkit/README.md) ·
[`motion_toolkit/README.md`](motion_toolkit/README.md) ·
[`planner/README.md`](planner/README.md).

---

## `outputs/`

### 먼저 볼 것 세 개

| 파일 | 내용 |
|---|---|
| **`open_loop_replay_limit.md`** | **핵심 결과.** 개루프 관절 재생의 한계를 측정하고 그것이 실험 설계에 무엇을 강제하는지까지 정리한 문서. WALK만 살아남는 이유(끌개 vs 반발 극한주기), 살아남은 WALK도 벤치마크를 못 건너는 이유(선회 예산), 그래서 {WALK, JUMP} 라이브러리로는 계획된 비교가 성립하지 않는다는 결론, 그리고 미검증 TURN 클립이라는 복구 후보까지. §7은 `curated/`가 서버에 도착해야 답할 수 있었던 두 질문(재추출 게이트, 클립 평균화)의 결과 |
| **`jump_profile.csv`** + **`skill_profile.md` §3** | **점프 변위 측정.** 4다리 동기 스킬 8건의 이륙/비행/착지. 비행 중에는 `base_pos_*`가 다리 기구학 기반이라 무효이므로, 높이와 이륙 속도를 비행시간(`v_z0 = g·T/2`)과 푸시오프 임펄스로 이중 추정합니다. 결론: `front_jump`는 비행 0.451±0.028 s, 정점 상승 0.250±0.031 m, **수평 이동 26±4 mm** — 제자리 수직 도약. `front_pounce`는 탄도 비행이 아닌 런지 |
| **`skill_profile.csv`** | **36세션 × 104컬럼** 스킬 특성 원본 테이블. 보폭·듀티·접촉 위상·게이트 패턴·명령 대비 실측 속도·NaN 비율 등 세션당 한 행. 임계값과 스킬 능력 수치의 최종 출처이며, 아래 md 보고서들은 전부 이 CSV의 롤업입니다 |

### 지형 분석

| 파일 | 내용 |
|---|---|
| `terrain_profile.csv` | 1600행 = 20태스크 × 10난이도 × 8구간. 구간별 최대 단차/갭/경사/통로폭/횡오프셋 |
| `task_summary.csv` | 위를 태스크 20개로 압축 (난이도 하/상/전체별) |
| `calibration_plan.md` | 프로브 지형 42개와 각각이 어떤 `CALIBRATION_NEEDED` 파라미터를 확정하는지. 벤치마크에서 임계값을 튜닝하지 않기 위한 별도 지형 |

### 스킬 분석

| 파일 | 내용 |
|---|---|
| `skill_profile.md` | `skill_profile.csv`의 스킬별 롤업. 듀티비가 분리 축(0.31 러닝트롯 / 0.52 트롯 / 0.64 슬로우워크), 비행상은 듀티 0.40 아래에서만 |
| `skill_transition.md` | 67개 `skill_send` 이벤트의 전환 비용. `skill_send`→실제 움직임 4.06±0.28 s(그중 ~2.4 s는 `run.sh` 시작), 전환 후 자세 안정화 중앙값 0.21 s |
| `skill_clips.md` | 동결된 재생 클립 4개의 요약 — 출처 세션, 샘플레이트, 듀티, 루프 이음매, 에일리어싱. 게인 스케줄이 스킬마다 다르다는 것도 여기서 드러납니다 |
| `gain_feasibility.md` | 런타임에 관절 게인을 바꿀 수 있는가 (답: 가능, 이 fork의 액추에이터는 explicit PD) |

### 플래너 오프라인

| 파일 | 내용 |
|---|---|
| `planner_offline.md` | 7200런 스윕 보고서 (`SWITCH_DELAY` 9종 × `STEP_WALK_MAX` 3종 × 20태스크 × 10난이도). **평가가 아니라 스킬 시퀀스 생성기입니다** — 넘어졌는지는 판정하지 않습니다 |
| `planner_offline_summary.csv` | 런당 한 행 (7200행). 스위치 수, 점프 수, 스킬별 틱 비율, 시퀀스 |
| `planner_offline_segments.csv` | 구간 단위 상세 |
| `planner_offline_unsupported.csv` | 대응 스킬이 없어 실패한 지점의 사유와 위치. 예외처리로 빼지 않고 전부 시도한 기록 |

### 시뮬 검증

| 파일 | 내용 |
|---|---|
| `server_day1.md` / `server_day1_results.md` | 서버 첫날 계획과 그 결과 (관절 순서·부호 규약 확정 포함) |
| `harness_findings.md` | 시뮬 없이 작성된 코드가 실제 Isaac Lab을 만났을 때 깨진 5가지. **그중 4개가 조용히 실패했고 2개는 자신만만한 오진단을 출력했습니다** |
| `sim_settings_audit.json` | 하네스의 모든 시뮬 설정을 upstream env config와 대조한 결과 |
| `isaac_actuator_probe.json` | 설치된 Isaac Lab이 실제로 허용하는 것 (액추에이터 클래스, 런타임 게인 쓰기 가능 여부, 제어 주기) |
| `replay_verify.csv` | `verify_skill_replay.py` 실행 결과 누적 — 클립, 재생 모드, 힙 부호, 시작 위상, 인계 상태, 생존 시간, 판정 |

---

## 현재 상태 (2026-08-28)

- **개루프 재생은 WALK만 안정적입니다.** WALK(듀티 0.64)는 60사이클 / 43.9 s / 10.7 m 동안 넘어지지
  않고, 롤 외란이 ±4°로 유계에 머무릅니다 — 끌개 극한주기. TROT(듀티 0.52)과 RUN(듀티 0.31)은
  사이클당 롤이 3–5배씩 증폭해 각각 **2.19 s / 2.47 s 만에 전복**하고, 그 전에 이미 기하학적
  지지각을 넘습니다(0.72 m / 0.41 m). 원인은 누적 오차가 아니라 발산입니다: 클립에는 관절 각도만
  있고 베이스 상태가 없어서, 실 로봇이 닫고 있던 루프를 재생이 대신 닫을 수 없습니다. 클립 비대칭도,
  위상 평균화도 원인이 아님을 각각 별도로 확인했습니다.
- **살아남은 WALK도 벤치마크를 건너지 못합니다.** 개루프 WALK는 11.8 °/m로 휘고, 벤치마크가
  허용하는 곡률은 최선의 조준을 가정해도 0.565 °/m입니다. **8개 goal 중 0개 도달** — 첫 goal을
  허용오차의 2.5배로 놓칩니다. 실 로봇 자신의 곡률(2.05 °/m)조차 예산을 3.6배 초과하므로, 이건 클립을
  더 잘 잘라서 해결될 문제가 아니라 **주행 중 헤딩 보정이 구조적으로 필요하다**는 뜻입니다.
- **`front_jump`은 제자리 수직 도약입니다.** 수평 이동 26±4 mm, 즉 **갭 통과 ≈ 0 m**. 전진 속도를
  가진 도약이 로그 라이브러리에 없습니다.
- **그래서 실험 설계를 재검토 중입니다.** 사용 가능 스킬이 {WALK, JUMP}로 줄면 Rule-Planner와
  Single-skill 하한선이 벤치마크 런의 **81.5%에서 구조적으로 동일한 정책**이 되고, 양쪽 다 0점을
  받습니다 — 바닥과 측정값이 둘 다 0인 비교는 아무것도 측정하지 못합니다. 또 하한선으로 명세된
  "고정 트롯"이 애초에 사용 불가입니다. 복구 후보는 로그에 이미 있습니다: `turn_right_20260824_223951`은
  듀티 0.715로 WALK보다 높고 비행상이 없어, 지금까지의 순서(듀티가 높을수록 유리)에서 안전한 쪽에
  있습니다. 검증되면 라이브러리가 {WALK, TURN, JUMP}가 되고 셀당 ~13회의 실질적 판단이 생깁니다.
  이건 예측이지 결과가 아닙니다. 미결 사항은 `outputs/open_loop_replay_limit.md` §6.4에 있습니다.

---

## 재현

**시뮬레이터가 필요 없는 것** (노트북에서 그대로 실행됩니다). `curated/`가 필요한 것은 표시해 두었습니다.

```bash
python scripts/freeze_benchmark.py --verify          # 동결 지형 재현성 확인
python scripts/profile_terrains.py                   # -> outputs/terrain_profile.csv
python scripts/profile_skills.py                     # curated/ 필요 -> skill_profile.*, jump_profile.csv
python scripts/simulate_planner_offline.py           # -> outputs/planner_offline*
python scripts/analyze_heading_budget.py             # 선회 예산 (순수 기하)
python scripts/analyze_drift.py --clips              # 클립 대칭성
python scripts/gate_reextract.py                     # curated/ 필요 — 클립 정의를 바꾸기 전 필수 게이트
```

**시뮬레이터가 필요한 것.** Isaac Lab은 **서버(z4)에만, Docker 이미지로만** 있습니다 —
네이티브 설치본은 없습니다. 모든 Isaac 스크립트는 `scripts/isaac_docker_run.sh`를 통해
`nvcr.io/nvidia/isaac-lab:3.0.0-beta2-post1` 컨테이너 안에서 돌아갑니다. 이 러너가
엔트리포인트·uid·HOME·PYTHONPATH·`CUBLAS_WORKSPACE_CONFIG`를 맞춰 주고, 프로젝트와
읽기 전용 upstream을 바인드 마운트합니다. `GPU=0|1` 환경변수로 GPU를 고를 수 있습니다(기본 1).

WALK 클립을 60사이클 재생하고 트레이스를 남기는 예:

```bash
scripts/isaac_docker_run.sh scripts/verify_skill_replay.py --clip WALK --cycles 60 \
    --headless --device cpu --hip-sign keep --contact-threshold-n 30 \
    --start-phase level --settle-mode stand --trace-npz outputs/traces/WALK_long.npz

python scripts/analyze_drift.py --trace outputs/traces/WALK_long.npz
```

`--hip-sign keep`, `--contact-threshold-n 30`, `--start-phase level`, `--settle-mode stand`는
서버 2일차에 확정된 설정입니다. 다른 값으로 돌린 결과는 위 문서들의 수치와 비교할 수 없습니다.
`--self-test` / `--explain`은 Isaac Lab 없이 동작합니다.

---

## GitHub에 없는 것

리포를 클론해도 아래는 따라오지 않습니다. 필요하면 별도로 전달받으세요.

| 없는 것 | 크기 | 비고 |
|---|---|---|
| `curated/` 원본 로그 36세션 | **542 MB** | 별도 전달. `~/projects/curated`에 두면 `terrain_toolkit/paths.py`가 찾습니다(`$GO2_CURATED_ROOT`로도 지정 가능). **읽기 전용** — 어떤 이유로도 수정하지 않습니다. 노트북과 z4 양쪽에 둡니다 |
| `outputs/video/` | | 재생 측면 영상(mp4)과 프레임 CSV |
| `outputs/traces/` | | 시뮬레이터 런 12개의 원시 트레이스 npz. `open_loop_replay_limit.md`의 모든 수치가 여기서 나옵니다 |
| `outputs/raw_cycle_ab/` 일부 | | §7.2 A/B의 런별 결과 CSV |
| `outputs/reextract_gate/` | | 재추출 게이트가 재빌드한 아카이브 사본 (검사용) |
| `logs/` | | 콘솔 로그와 위상/규약 실험 트레이스 |
| upstream 리포 | | `eurekaverse-go2-parkour` — 별도 체크아웃. **읽기 전용**, 패치가 필요하면 `go2-planner/patches/`에 파일로 만듭니다 |

`data/*.npz`는 예외적으로 커밋되어 있습니다 (동결본이자 임계값 근거 자료). 해시 파일
(`*.sha256`, `*.meta.json`)로 검증하고, 재생성하지 마세요.
