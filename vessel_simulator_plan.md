선박 시뮬레이터 개발 기획
1. 프로젝트 기획 (Planning)
1.1. 프로젝트 목표 (Goal)
핵심 목표: 선박의 실제 운동에 가까운 물리 운동역학을 모델에 적용하여, 다양한 해양 환경(수심, 장애물 등)과 상호작용하는 실제 선박의 항해에 가까운 시뮬레이터를 구현 한다.

연구 및 확장성: 객체지향 설계를 통해 각 모듈(선박, 물리엔진, 환경 등)의 독립성을 확보하여, 향후 특정 기능(자율운항 알고리즘 테스트, 특정 해역 시뮬레이션 등)을 쉽게 추가하고 연구에 활용할 수 있도록 한다.

1.2. 핵심 기능 정의 (Core Features)
선박 모델 (Vessel Model):

선박의 기본 제원(길이, 폭, 흘수, 배수량) 정의

추진 시스템(프로펠러) 및 조향 시스템(러더) 모델링

선박의 현재 상태(위치, 속도, 선수각, 각속도) 데이터 관리

물리 엔진 (Physics Engine):

1단계 (기본): 기본 6자유도 운동(6-DOF) 방정식 구현

2단계 (유체역학): 저항, 추진력, 조종력 등 유체역학적 힘 계산 모델 (MMG 모델 등 참조)

3단계 (환경 영향): 얕은 수심(Shallow Water)에서의 운동 변화, 구속 수로 효과 등 환경에 따른 동적 변화 계산

해양 환경 모델 (Environment Model):

수심 데이터 (Bathymetry): 특정 해역의 수심 정보를 2D 그리드 형태로 관리

안전 수심선 (Safety Contour): 선박의 흘수를 기준으로 항해 가능/불가능 영역 구분

장애물 (Obstructions): 고정된 장애물(암초, 섬) 및 이동 장애물(타 선박) 배치

데이터 연동 (Data Integration):

AIS 데이터: 더미 AIS(Automatic Identification System) 데이터를 파싱하여 시뮬레이션에 타 선박으로 등장시키는 기능

시각화 (Visualization):

1단계 (기본): 2D Top-down 뷰에서 선박의 궤적, 수심, 장애물 등을 시각화

2단계 (고급): 선박의 상태(속도, RPM 등)를 표시하는 GUI 추가

1.3. 개발 단계 (Roadmap)
1단계: 코어 모델링 (Core Modeling)

Vessel 클래스 정의 (기본 제원 및 상태)

벡터, 좌표계 등 기본 유틸리티 모듈 구현

시뮬레이션의 시간 흐름을 제어하는 기본 Simulator 루프 구현

2단계: 기본 물리 엔진 구현 (Basic Physics)

외부 힘 없이 관성에 의해서만 움직이는 선박 구현

엔진 및 러더 입력에 따라 추력/조종력이 단순하게 발생하는 모델 구현

3단계: 환경 모델 및 상호작용 구현 (Environment Interaction)

수심 데이터를 로드하고 관리하는 Environment 클래스 구현

선박과 수심의 상호작용 (Shallow Water Effect) 기초 모델 적용

고정 장애물과의 충돌 감지 로직 구현

4단계: 데이터 연동 및 고급 기능 (Data Integration & Advanced Features)

AIS(CSV 형태) 데이터 파서 구현

시뮬레이션에 타 선박(AIS 데이터 기반) 추가

5단계: 시각화 및 UI (Visualization & UI)

Pygame 등을 이용한 2D 시각화 모듈 개발

시뮬레이션 제어 및 상태 모니터링을 위한 간단한 UI 구현

2. 폴더 구조 제안 (Folder Structure)
모듈의 재사용성과 확장성, 그리고 사용자 편의성을 극대화하기 위해 상세화된 폴더 구조를 제안합니다. 이 구조는 시뮬레이션의 핵심 라이브러리와 사용자 인터페이스를 명확히 분리합니다.

vessel_dynamics_simulator/
│
├── vds/                          # 핵심 시뮬레이션 라이브러리 (패키지)
│   ├── __init__.py
│   │
│   ├── core/                     # 시뮬레이터의 기본 구성 요소
│   │   ├── __init__.py
│   │   ├── simulator.py          # 메인 시뮬레이션 루프, 시간 관리
│   │   └── kinematics.py         # 운동학 모델 (위치, 속도, 좌표 변환)
│   │
│   ├── models/                   # 선박 및 물리 모델
│   │   ├── __init__.py
│   │   ├── vessels/              # 선박 모델 패키지
│   │   │   ├── __init__.py
│   │   │   ├── base_vessel.py    # 모든 선박의 추상 기본 클래스
│   │   │   └── kcs_container.py  # KCS 컨테이너 선박 모델 예시
│   │   │
│   │   └── dynamics/             # 운동 역학(물리) 모델 패키지
│   │       ├── __init__.py
│   │       ├── base_model.py     # 물리 모델의 추상 기본 클래스
│   │       └── mmg_model.py      # MMG(Maneuvering Modeling Group) 모델
│   │
│   ├── environment/                # 해양 환경 모델
│   │   ├── __init__.py
│   │   ├── wind.py               # 바람 모델
│   │   ├── waves.py              # 파도 모델
│   │   ├── current.py            # 조류 모델
│   │   └── geography.py          # 지형 (수심, 장애물) 관리
│   │
│   ├── data_handler/             # 데이터 로딩 및 파싱
│   │   ├── __init__.py
│   │   ├── ais_parser.py         # AIS 데이터 파서
│   │   └── chart_loader.py       # 해도, 수심 데이터 로더
│   │
│   └── utils/                    # 공통 유틸리티
│       ├── __init__.py
│       ├── constants.py          # 물리 상수 (중력, 해수 밀도 등)
│       └── conversions.py        # 단위 변환 (노트-m/s 등)
│
├── app/                          # 사용자 인터페이스(UI) 애플리케이션
│   ├── __init__.py
│   ├── main_window.py            # 메인 애플리케이션 윈도우
│   ├── renderer.py               # 시뮬레이션 상태 시각화(렌더링)
│   └── widgets/                  # 커스텀 UI 위젯 (버튼, 슬라이더 등)
│
├── scenarios/                    # 시뮬레이션 시나리오 정의
│   ├── configs/                  # 시나리오 설정 파일 (YAML, JSON 등)
│   │   └── turning_test_kcs.yaml
│   └── run_scenario.py           # 설정 파일을 읽어 시나리오를 실행하는 스크립트
│
├── data/                         # 원본 데이터 파일
│   ├── ais/
│   │   └── sample_track.csv
│   ├── bathymetry/
│   │   └── busan_harbor.csv
│   └── vessel_params/
│       └── kcs_hydrodynamics.json # KCS 선박의 유체동역학 계수
│
├── tests/                        # 단위 및 통합 테스트
│   ├── test_kinematics.py
│   └── test_mmg_model.py
│
├── docs/                         # 프로젝트 문서
│   └── model_equations.md        # 사용된 수학 모델에 대한 설명
│
├── main.py                       # 애플리케이션 실행 진입점
├── requirements.txt              # 프로젝트 의존성 라이브러리
└── README.md                     # 프로젝트 개요 및 사용법

3. 프레임워크 및 기술 스택 (Framework & Tech Stack)
프로그래밍 언어: Python 3.10+

이유: 과학 계산 및 데이터 분석 라이브러리가 풍부하고, 객체지향 프로그래밍에 적합하며 빠른 프로토타이핑이 가능합니다.

핵심 라이브러리:

NumPy: 모든 벡터 및 행렬 연산의 기반이 됩니다. 선박의 운동을 계산하는 데 필수적입니다.

SciPy: 미분 방정식 해결 등 복잡한 과학 계산에 사용될 수 있습니다.

Pandas: AIS나 수심 같은 테이블 형태의 데이터를 읽고 처리하는 데 매우 유용합니다.

시각화 라이브러리 (UI/UX):

Pygame 또는 PyQt / PySide

Pygame: 2D 기반의 실시간 시뮬레이션 및 상호작용 구현에 적합합니다. 빠른 프로토타이핑에 유리합니다.

PyQt/PySide: 버튼, 슬라이더, 메뉴 등 복잡하고 전문적인 GUI 애플리케이션을 구축하는 데 강력한 기능을 제공합니다.

데이터 분석 및 결과 시각화:

Matplotlib, Seaborn: 시뮬레이션이 끝난 후 선박의 궤적, 속도 변화 등을 그래프로 분석하고 시각화하는 데 유용합니다.

개발 도구:

버전 관리: Git, GitHub

테스트 프레임워크: pytest

4. 기타 고려사항 (Other Considerations)
좌표계 정의: 시뮬레이션에서 사용할 명확한 좌표계(예: NED - North-East-Down)를 초기에 정의해야 합니다. 이는 모든 물리 계산의 기준이 됩니다.

단위 테스트: 물리 모델이 복잡해질수록 각 계산 모듈의 정확성을 검증하는 단위 테스트의 중요성이 커집니다. 초기부터 테스트 코드를 작성하는 습관을 들이는 것이 좋습니다.

수학적 모델: 선박 운동 모델(예: MMG 모델)은 매우 복잡합니다. 초기에는 이를 단순화한 모델부터 시작하여 점차 실제에 가깝게 발전시키는 접근 방식이 유효합니다.

문서화: 각 클래스와 함수의 역할, 사용된 물리 공식 등을 주석(Docstring)이나 별도의 문서로 꾸준히 기록하여 프로젝트의 유지보수성을 높여야 합니다.

고급 설정 및 로깅 시스템:

설정 관리: 시뮬레이션의 정밀도(time step), 물리 상수, UI 테마 등 다양한 설정을 코드와 분리하여 관리하는 중앙 설정 시스템(예: config.yaml 파일 로더)을 도입합니다.

로깅: 시뮬레이션 진행 상황, 주요 이벤트(충돌 등), 오류 등을 파일로 기록하는 체계적인 로깅 시스템을 구축합니다. 이는 디버깅과 결과 분석에 필수적입니다.

의존성 관리 고도화:

Poetry나 Pipenv 같은 도구를 사용하여 개발 환경과 배포 환경의 의존성을 명확하게 분리하고, 버전 충돌 문제를 사전에 방지하는 것이 안정성 확보에 유리합니다.

지속적 통합 (Continuous Integration):

GitHub에 코드를 push할 때마다 pytest로 작성된 테스트 코드가 자동으로 실행되도록 GitHub Actions 같은 CI 도구를 연동하여 코드 품질을 일관되게 유지합니다.

플러그인 아키텍처 (Plugin Architecture) 고려:

연구 활용도를 극대화하기 위해, 사용자가 직접 정의한 선박 모델이나 동역학 모델을 코어 라이브러리(vds) 수정 없이 쉽게 추가하고 테스트할 수 있는 플러그인 형태의 구조를 고려할 수 있습니다.