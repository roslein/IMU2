from abc import ABC, abstractmethod

class SensorDriver(ABC):
    @abstractmethod
    def connect(self) -> bool:
        """통신 또는 파일 리소스를 개방하고 준비 상태를 확인하여 성공 여부를 반환합니다."""
        pass

    @abstractmethod
    def fetch_raw_data(self) -> dict:
        """9축 센서의 원시 성분 및 동기화 데이터를 딕셔너리 형태로 획득해 반환합니다."""
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """포트 또는 파일 로드를 해제하고 메모리 리소스를 안전하게 회수합니다."""
        pass

class IMUVisualizer(ABC):
    @abstractmethod
    def init_plot(self) -> None:
        """Matplotlib 3D Canvas 및 축 좌표계 영역을 초기화 셋업합니다."""
        pass

    @abstractmethod
    def update_plot(self, collected_faces: set, normals: list, new_point: list = None, new_matched_idx: int = None, expected_face_idx: int = None) -> None:
        """현재 거치된 자세 정보와 구면 법선을 매초 갱신해 렌더링합니다."""
        pass
