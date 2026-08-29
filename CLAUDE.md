# go2-planner

Go2 파쿠르: 룰베이스 스킬 플래너 vs E2E 학습 정책 비교 (Isaac Lab 시뮬레이션)

## 0. 이 파일의 역할

작업 시작 전 반드시 읽는다. 여기 적힌 원칙은 개별 작업 지시보다 우선한다.
지시와 이 문서가 충돌하면 진행하지 말고 사용자에게 확인한다.

---

## 1. 경로 규칙

| 경로 | 권한 | 내용 |
|---|---|---|
| `~/projects/eurekaverse-go2-parkour` | **읽기 전용** | upstream 리포. 서버(z4)에서는 `~/ev-go2` 로 가는 심링크 |
| `~/projects/curated` | **읽기 전용** | 실 로봇 텔레옵 로그 36세션. 노트북과 z4 양쪽에 둔다 |
| `~/projects/go2-planner` | 읽기/쓰기 | 이 프로젝트. 모든 생성·수정은 여기서만 |

- 원본 두 곳은 어떤 이유로도 수정하지 않는다. 패치가 필요하면 `go2-planner/patches/`에 파일로 만든다.
- **eurekaverse: 서버에서는 `~/ev-go2` 심링크다. `chmod` 잠금을 걸지 않고 읽기만 하며, 해당
  트리에는 어떤 수정도 하지 않는다.** 잠금을 걸지 않는 이유는 `~/ev-go2` 가 다른 사람이
  distillation에 쓰는 살아 있는 작업 트리이기 때문이다 — 잠그면 그쪽 산출물 기록이 막힌다.
  쓰기 권한이 있다는 사실은 쓰라는 뜻이 아니다. 규율로 지킨다.
- **`curated/` 는 z4에도 둔다** (2026-08-28 변경. 이전 규칙은 "서버에 없어도 된다"였다).
  바뀐 이유는 클립 정의가 더 이상 확정이 아니기 때문이다 — 기동 구간을 클립에 포함할지는
  시뮬 재생 결과로만 판정되고, 그 판정을 내리는 Isaac Lab은 z4에만 있다. 원본을 유일한
  계측기에서 떼어 놓으면 한 번 자를 때마다 노트북 추출 → 동결 → 전송이 된다.
- **재추출 게이트.** `curated/` 가 z4에 도착하면 클립 정의를 건드리기 **전에**
  `scripts/extract_skill_clips.py` 를 수정 없이 한 번 돌려 `data/skill_clips.meta.json` 의
  `npz_sha256` (현재 `aab85a03…`) 가 그대로 재현되는지 확인한다. `np.savez_compressed` 는
  zip 타임스탬프를 0으로 쓰므로 내용이 같으면 파일 해시도 같다 — 즉 이 비교는 유효하다.
  재현되지 않으면 그 사실을 먼저 규명한다. 재현 여부를 모른 채 클립 정의를 바꾸면 새 아카이브가
  두 가지 이유로 달라지고 어느 쪽도 분리되지 않는다.
- 경로는 `pathlib` + `terrain_toolkit/paths.py`를 쓴다. 하드코딩 금지.

---

## 2. 실험 설계 원칙

### 비교 구도

| 실험군 | 스킬 | 선택 방식 | 역할 |
|---|---|---|---|
| E2E | 연속 정책 | 학습 | 상한 |
| Rule-Planner | 이산 스킬 | 룰 | 측정 대상 |
| Single-skill | 트롯 고정 | 없음 | 하한 |
| (선택) Learned-Planner | 이산 스킬 | 학습 | 판단 방식만 다른 대조군 |

- Single-skill 하한선은 필수다. 없으면 Rule-Planner 점수가 해석되지 않는다.

### 지형

- **평가 지형: benchmark 20태스크.** 학습 지형에서 평가하지 않는다 (E2E에 유리해짐).
- 동결본 `data/benchmark_frozen.npz`를 정본으로 쓴다. 재생성 금지.
- `height_fields_before_fix`가 정본. upstream의 benchmark 경로는 `fix_terrain`을 호출하지 않으므로 시뮬이 실제로 쓰는 것과 일치한다.
- 해시 검증: `data/benchmark_frozen.sha256`, `data/upstream_commit.txt`

### 금지 사항

- **임계값을 성능 데이터로 조정하지 않는다.** 지형 기하 분포와 스킬 실측 능력에서만 도출한다.
  - 성능 기반 보정이 필요하면 benchmark가 아닌 별도 캘리브레이션 지형을 쓴다.
- **upstream 버그를 고치지 않는다.** E2E도 같은 버그를 겪으므로 양쪽에 동일 적용되어야 공정하다.
  - `sphere_bump`, `sphere_bump_lips`, `flat_circle_jump`, `bump_jump`: int16 오버플로 → 먼 곳 가짜 블록 + spawn 잡셀
  - `set_terrain_stepping_stones_flat`: 디스패처 미등록 (21개 정의 / 20개 사용). 20종으로 진행
  - `staircase_spiral`: `difficulty_scaling` 미사용 → 난이도 10단계가 사실상 동일할 수 있음
- **대응 스킬이 없는 태스크를 예외처리로 빼지 않는다.** 20개 전부 동일하게 돌려 결과가 0으로 나오게 둔다. "시도했으나 안 됨"이 결과다.
- **전환 지연·임계값을 코드에 상수로 박지 않는다.** 전부 config로 노출한다.

---

## 3. 확정된 사실 (실측)

### 지형 (`outputs/terrain_profile.csv`, 1600행)

- 20태스크 × 난이도 10 × goal 구간 8
- 셀 18m × 4m, `horizontal_scale` 0.05m, `vertical_scale` 0.005m, goal 8개
- 구간 길이 약 2.25m
- 동결본은 upstream 대비 `random_uniform_terrain` 노이즈와 셀 테두리 0.5m 패드가 미적용. README에 명시할 것.

### 스킬 (`outputs/skill_profile.csv`, `outputs/jump_profile.csv`)

- 36세션 중 29개가 대각선 트롯. 선회·횡이동·후진도 전부 트롯 변형.
- 듀티비가 분리 축: 0.31(러닝 트롯) / 0.52(트롯) / 0.64(슬로우 워크). 비행상은 듀티 0.40 아래에서만 (r = −0.95).
- 명령 ≠ 실측: `move x=1.5~2.0` + `speed_level 0` → 실측 0.48 m/s (odometry 0.40).
- `front_jump`: 비행 0.451±0.028s, 정점 상승 0.250±0.031m, **수평 이동 26±4mm**. 제자리 수직 도약.
- `front_pounce`: 탄도 비행 아님. 앞다리만 뻗는 런지. `flight`가 아니라 `all-off`로 표기.
- 장애물 한계: step-up 이론상 0.25m, 실용 추정 0.12~0.15m. **갭 통과 ≈ 0m** (전진 속도를 가진 도약이 라이브러리에 없음).
- PosStopF 비율 ≤0.02% (이동 스킬 전부) → 관절 궤적 직접 재생 가능. `stand_down`→`recovery_stand`만 23%.

### 센서 (`legged_robot_config.py: CustomDepthCfg`)

- `use_camera = False`가 기본값. 플래너에서 depth를 쓰려면 켜야 한다.
- depth 켜면 `camera_num_envs = 192`, 지형 격자 10×20으로 축소됨 → 평가 시간 증가.
- 마운트: 앞 0.272m, 위 0.092m, 아래로 0.52 rad (29.8°)
- FOV 87° (크롭 후 약 78°), 해상도 90×60 (D435 640×360 ÷6, 좌우 8px 크롭)
- **`near_clip = 0`, `far_clip = 2`** → 유효 범위 약 0.25~2.0m. 2m 너머는 보이지 않는다.
- 갱신 10Hz (`update_interval=5`), 지연 0.02s (`depth_delay_steps=1`)
- 노이즈: `granular_noise` 0.02, `blackout_noise` 0.03. blur/erase/bias는 0 (꺼짐).
- 해상도 한계: 픽셀당 지면 폭이 0.7m에서 약 2.7cm, 2.0m에서 약 24cm. **먼 거리 판단은 신뢰도가 낮다.**

### 전환 비용 (`outputs/skill_transition.md`)

- `skill_send` → 실제 움직임: 4.06 ± 0.28s
- 그중 약 2.4s는 `run.sh` 시작 오버헤드 (MANIFEST 노트). 나머지 약 1.6s는 출처 미확인.
- 전환 후 자세 안정화: 0.21s 중앙값, 1.63s 최악
- `front_jump`/`front_pounce` 앞에 `balance_stand`가 8/8 선행. 시뮬에서는 펌웨어 제약이 아니라 **초기 자세 정합 문제**로 남는다 (녹화 궤적이 정지 자세에서 시작).
- 시뮬에는 `run.sh`가 없다. 기본값은 실측 안정화 시간 0.21s를 쓰고, config로 노출해 민감도 분석(0 / 0.21 / 2.4 / 4.06)을 돌린다.

---

## 4. 파생 제약

- **lookahead = base_margin + SWITCH_DELAY × speed**
  - 0.21s → 0.10m / 2.4s → 1.15m / 4.06s → 1.95m
  - `far_clip = 2` 이므로 지연 상한은 실용적으로 약 2.5s
  - lookahead가 센서 범위를 넘으면 경고를 낸다. 조용히 실패하지 않게.
- 사각지대 0~0.25m: 발밑은 보이지 않는다. 플래너는 직전 관측을 유지해야 한다.
- 판단 신뢰 구간은 약 1m 이내. 그 너머는 해상도가 뭉개진다.

---

## 5. 열린 질문

- 전환 지연을 시뮬에서 얼마로 잡을지 (대학원생분 확인 대기)
- 4.06s 중 `run.sh` 2.4s 외 1.6s의 출처
- E2E가 `horizontal_scale`을 0.05로 돌리는지 (depth 학습 시 0.1 권장 주석 있음 → 0.1이면 동결본과 어긋남)
- 시뮬에서 전진 중 `front_jump` 발사 시 관성으로 갭을 넘는지 (녹화는 정지 출발이라 26mm)
- 평가 시 depth 노이즈 증강을 켤지 (양쪽 군 동일 적용이면 무방)

---

## 6. 데이터 취급 주의

`curated/` 로그를 다룰 때:

- 모션 구간을 `events.jsonl`의 `skill_send` 시각으로 잡지 않는다. 관절 속도로 판정한다.
- `foot_pos_*`, `foot_vel_*`는 전부 0 (펌웨어 미지원). 사용 금지.
- `contact_*` 컬럼의 고정 20N 임계값을 쓰지 않는다. 세션별·다리별로 산출한다 (실측 24~51N).
- 공중에서는 `base_pos_*`, `base_v*`, `body_height`가 다리 기구학 기반이라 무효다. 비행 중 값은 비행시간·임펄스로 우회 추정한다.
- NaN 존재 (상태 ≤0.34%, `q_des` ≤1.23%). nan-safe 연산을 쓴다.

---

## 7. 커밋

- 논리 단위로 나눠 커밋한다.
- `.npz` 등 대용량은 gitignore, 해시 파일과 CSV 요약은 커밋한다 (임계값 근거 자료).
- **커밋 메시지에 `Co-Authored-By:` 나 "Generated with Claude Code" 류의 서명을 넣지 않는다.**
  도구 이름은 이 리포의 기록이 아니다. 무엇을 왜 바꿨는지만 남긴다.
- **작성자는 `user.name = yunseo904`, `user.email = yunseotwo@gmail.com` 으로 고정한다.**
  전역 `~/.gitconfig` 에는 사용자 정보가 없으므로 리포 로컬 설정이 정본이다. 새로 클론하면
  가장 먼저 다음을 실행한다:

  ```bash
  git config user.name  yunseo904
  git config user.email yunseotwo@gmail.com
  ```

  커밋 전에 `git log -1 --format='%an <%ae>'` 로 확인한다. 다른 주소로 올라간 커밋은
  GitHub 잔디와 작성자 귀속이 어긋나므로 뒤늦게 고치려면 히스토리를 다시 써야 한다.
