ARG PLATFORM=linux/amd64
ARG BINARY=ur3e_mockup_linux_x86_64
FROM --platform=${PLATFORM} ubuntu:23.10

ARG BINARY=ur3e_mockup_linux_x86_64
WORKDIR /app
COPY ${BINARY} /app/ur3e_mockup
RUN chmod +x ur3e_mockup

CMD ["./ur3e_mockup"]
