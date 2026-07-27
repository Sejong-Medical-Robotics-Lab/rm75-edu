# 제3자 라이선스 고지

## RealMan ros2_rm_robot 발췌 (`rm_description`·`rm_ros_interfaces`·`rm_driver`)

- 출처: [RealManRobot/ros2_rm_robot](https://github.com/RealManRobot/ros2_rm_robot)
  — 교재 3.1 "제조사 제공 드라이버·컨트롤러, 배부 저장소에 포함"에 따라
  발췌 재배포. `rm_description` 은 rm_75 관련 urdf·meshes 만 남긴 슬림판
  (CMakeLists 만 그에 맞게 수정, urdf·메시 원본 무수정),
  `rm_ros_interfaces`·`rm_driver` 는 원본 그대로.
- **라이선스 표기 주의**: 검토 시점 기준 업스트림 저장소 루트에 LICENSE
  파일이 없고, 해당 package.xml 의 라이선스 필드가 "TODO: License
  declaration" 상태입니다. 즉 재배포 조건이 명시되어 있지 않습니다.
  - 연구실 내부 교육용 배부는 통상 문제 소지가 낮으나, **저장소를 공개
    GitHub 에 게시하기 전에는 제조사(RealMan) 확인을 권장**합니다.
  - 대안: 제조사 패키지를 저장소에 포함하는 대신, README 에
    `git clone https://github.com/RealManRobot/ros2_rm_robot` (또는 vcs
    import) 절차로 학생이 직접 받게 하는 방식으로 전환할 수 있습니다 —
    교재의 "배부 저장소에 포함" 문구만 그에 맞게 조정하면 됩니다.

## MoveIt 설정의 유래

`rm75_edu_moveit_config` 는 위 저장소의 `rm_moveit2_config/rm_75_config`
(BSD 표기, MoveIt Setup Assistant 자동 생성물)를 기반으로 그리퍼·TCP 를
추가한 파생본입니다. `moveit.rviz`·`pilz_cartesian_limits.yaml`·
`joint_limits.yaml`(팔 7관절 부분)은 원본을 복사·확장했습니다.
