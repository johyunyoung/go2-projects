FROM johyunyoung/go2-deploy:latest

ARG USERNAME=user
ARG UID=1000
ARG GID=1000

# 1) UID/GID 재사용 또는 생성 + 홈 디렉터리 준비
RUN set -eux; \
    gname="$(getent group ${GID} | cut -d: -f1 || true)"; \
    if [ -z "${gname}" ]; then \
      gname="${USERNAME}"; groupadd -g "${GID}" "${gname}"; \
    fi; \
    uname="$(getent passwd ${UID} | cut -d: -f1 || true)"; \
    if [ -z "${uname}" ]; then \
      uname="${USERNAME}"; useradd -m -u "${UID}" -g "${GID}" -s /bin/bash "${uname}"; \
    fi; \
    mkdir -p /home/${USERNAME}; \
    chown -R ${UID}:${GID} /home/${USERNAME}

# 2) sudo 설치: root 비번 요구(rootpw) + user 개별 허용(비번 필요)
RUN set -eux; \
    apt-get update; \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends sudo; \
    sed -ri 's/^\%sudo\b/#&/; s/^\%admin\b/#&/' /etc/sudoers; \
    echo 'Defaults rootpw' >> /etc/sudoers; \
    realuser="$(getent passwd ${UID} | cut -d: -f1)"; \
    printf '%s ALL=(ALL) ALL\n' "${realuser}" >/etc/sudoers.d/90-user; \
    chmod 0440 /etc/sudoers.d/90-user; \
    visudo -cf /etc/sudoers; \
    rm -rf /var/lib/apt/lists/*

# 3) 기본 실행 사용자(비-root)
USER ${UID}:${GID}
ENV HOME=/home/${USERNAME}
WORKDIR /home/${USERNAME}

