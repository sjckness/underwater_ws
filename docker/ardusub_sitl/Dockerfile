# ArduSub SITL for the HIL setup (M8a/M8b prerequisite). Unforked ArduPilot at a pinned release tag, SITL board only.
#
#   docker build -t uw-ardusub:4.5.7 docker/ardusub_sitl
#
# Tag: ArduSub-4.5.7, the last release of the 4.5 line (the tags are named "ArduSub-4.x.y" up to 4.5; "Sub-4.x.y"
# starts at 4.7.0). To move to another release, change only ARDUPILOT_TAG.
#
# Layers are ordered so a rebuild after a change touches as little as possible: apt and pip (never change), then the
# source clone (changes with the tag), then the compile, then the small runtime image (entrypoint changes only here).
# Two stages: the toolchain (~1 GB) stays in the build stage, the runtime image holds the ardusub binary and the
# stock default-parameter files.

ARG UBUNTU=22.04

# ---------------------------------------------------------------------------------------------------- build stage
FROM ubuntu:${UBUNTU} AS build
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
        build-essential ca-certificates git gawk python3 python3-pip python3-dev python3-lxml python3-setuptools \
 && rm -rf /var/lib/apt/lists/*
# empy 3.3.4 is the version ArduPilot's build requires (empy 4 breaks it)
RUN pip3 install --no-cache-dir empy==3.3.4 future pexpect

ARG ARDUPILOT_TAG=ArduSub-4.5.7
RUN git clone --depth 1 --branch "${ARDUPILOT_TAG}" --recurse-submodules --shallow-submodules --jobs 8 \
        https://github.com/ArduPilot/ardupilot.git /src/ardupilot \
 && git -C /src/ardupilot rev-parse HEAD > /src/commit.txt

WORKDIR /src/ardupilot
RUN ./waf configure --board sitl \
 && ./waf sub -j"$(nproc)"

# ------------------------------------------------------------------------------------------------- runtime stage
FROM ubuntu:${UBUNTU} AS runtime
ENV DEBIAN_FRONTEND=noninteractive

# non-root: uid 1000, no login shell, the only writable place is its home (eeprom.bin, logs, terrain)
RUN useradd --create-home --uid 1000 --shell /usr/sbin/nologin ardu

COPY --from=build /src/ardupilot/build/sitl/bin/ardusub                        /opt/ardupilot/bin/ardusub
COPY --from=build /src/ardupilot/Tools/autotest/default_params/sub.parm        /opt/ardupilot/default_params/sub.parm
COPY --from=build /src/ardupilot/Tools/autotest/default_params/sub-6dof.parm   /opt/ardupilot/default_params/sub-6dof.parm
COPY --from=build /src/commit.txt                                              /opt/ardupilot/COMMIT
COPY --chmod=755 entrypoint.sh                                                 /opt/ardupilot/entrypoint.sh

USER ardu
WORKDIR /home/ardu/sitl
RUN mkdir -p /home/ardu/sitl
ENTRYPOINT ["/opt/ardupilot/entrypoint.sh"]
