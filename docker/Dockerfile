# ROS 2 Galactic이 설치된 공식 이미지 사용
FROM osrf/ros:galactic-desktop

# 필요한 패키지 및 LLVM 저장소 추가
RUN apt-get update && \
    apt-get install -y wget gnupg software-properties-common lsb-release && \
    wget -qO - https://apt.llvm.org/llvm-snapshot.gpg.key | apt-key add - && \
    add-apt-repository "deb http://apt.llvm.org/focal/ llvm-toolchain-focal main" && \
    apt-get update && \
    apt-get install -y \
        build-essential \
        clang-10 \
        lld-10 \
        g++-7 \
        cmake \
        ninja-build \
        libvulkan1 \
        python \
        python-dev \
        python3-dev \
        python3-pip \
        libpng-dev \
        libtiff5-dev \
        libjpeg-dev \
        tzdata \
        sed \
        curl \
        unzip \
        autoconf \
        libtool \
        rsync \
        libxml2-dev \
        git && \
    update-alternatives --install /usr/bin/clang++ clang++ /usr/lib/llvm-10/bin/clang++ 180 && \
    update-alternatives --install /usr/bin/clang clang /usr/lib/llvm-10/bin/clang 180

# 파이썬 패키지 설치
RUN pip install --no-cache-dir setuptools && \
    pip3 install --no-cache-dir -Iv setuptools==47.3.1 && \
    pip install --no-cache-dir distro && \
    pip3 install --no-cache-dir distro && \
    pip install --no-cache-dir wheel && \
    pip3 install --no-cache-dir wheel auditwheel

# ROS 2 환경 설정
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc

# 워크스페이스 생성
WORKDIR /platoon_ws
RUN /bin/bash -c "source /opt/ros/galactic/setup.bash && cd /platoon_ws && colcon build"

# 기본 실행 셸
CMD ["bash"]
