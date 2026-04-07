#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WORK_ROOT="${WORK_ROOT:-$(cd "${REPO_ROOT}/.." && pwd)}"
LOG_DIR="${LOG_DIR:-/tmp/s32k-flexcan-smoke}"
MODE="${1:-all}"

QEMU_BIN="${QEMU_BIN:-${REPO_ROOT}/build-arm-s32k/qemu-system-arm}"
CLASSIC_ELF="${CLASSIC_ELF:-${WORK_ROOT}/S32K148_Project_FlexCan/Debug/S32K148_Project_FlexCan.elf}"
CANFD_ELF="${CANFD_ELF:-${WORK_ROOT}/S32K148_Project_CanFd/Debug/S32K148_Project_CanFd.elf}"
RXFIFO_ELF="${RXFIFO_ELF:-${WORK_ROOT}/validation/official_fifo/build/S32K148_Project_CanRxFifo.elf}"
TRIPLE_ELF="${TRIPLE_ELF:-${WORK_ROOT}/validation/s32k_flexcan_triple_demo.elf}"

mkdir -p "${LOG_DIR}"

QEMU_PID=""
CANDUMP_PID=""

cleanup() {
    set +e
    if [[ -n "${QEMU_PID}" ]]; then
        kill "${QEMU_PID}" 2>/dev/null || true
        wait "${QEMU_PID}" 2>/dev/null || true
        QEMU_PID=""
    fi
    if [[ -n "${CANDUMP_PID}" ]]; then
        kill "${CANDUMP_PID}" 2>/dev/null || true
        wait "${CANDUMP_PID}" 2>/dev/null || true
        CANDUMP_PID=""
    fi
}

trap cleanup EXIT

setup_vcan() {
    local dev

    for dev in vcan0 vcan1 vcan2; do
        ip link add "${dev}" type vcan 2>/dev/null || true
        ip link set up dev "${dev}"
    done
}

require_file() {
    local path="$1"

    if [[ ! -f "${path}" ]]; then
        echo "missing file: ${path}" >&2
        exit 1
    fi
}

require_pattern() {
    local pattern="$1"
    local file="$2"

    if ! grep -Fq -- "${pattern}" "${file}"; then
        echo "missing pattern '${pattern}' in ${file}" >&2
        exit 1
    fi
}

require_count() {
    local expected="$1"
    local pattern="$2"
    local file="$3"
    local actual

    actual="$(grep -F -c -- "${pattern}" "${file}")"
    if [[ "${actual}" != "${expected}" ]]; then
        echo "pattern '${pattern}' expected ${expected}, got ${actual} in ${file}" >&2
        exit 1
    fi
}

run_single_case() {
    local name="$1"
    local elf="$2"
    local inject_cmds="$3"
    local candump_log="${LOG_DIR}/${name}-candump.log"
    local qemu_log="${LOG_DIR}/${name}-qemu.log"

    cleanup
    candump -L vcan0 > "${candump_log}" 2>&1 &
    CANDUMP_PID=$!
    sleep 1

    "${QEMU_BIN}" \
        -display none \
        -machine s32k1xx-can,canbus0=canbus0 \
        -object can-bus,id=canbus0 \
        -object can-host-socketcan,id=hostcan0,if=vcan0,canbus=canbus0 \
        -kernel "${elf}" > "${qemu_log}" 2>&1 &
    QEMU_PID=$!

    sleep 2
    bash -lc "${inject_cmds}"
    sleep 3
    cleanup

    case "${name}" in
        official-flexcan)
            require_count 2 "vcan0 555#332211A577665544" "${candump_log}"
            require_count 1 "vcan0 511#0011223344556677" "${candump_log}"
            ;;
        official-canfd)
            require_count 2 "vcan0 555##5332211A577665544" "${candump_log}"
            require_count 1 "vcan0 511##50011223344556677" "${candump_log}"
            ;;
        official-rxfifo)
            require_count 4 "vcan0 555#332211A577665544" "${candump_log}"
            require_count 1 "vcan0 511#0011223344556677" "${candump_log}"
            require_count 1 "vcan0 511#8899AABBCCDDEEFF" "${candump_log}"
            require_count 1 "vcan0 511#1021324354657687" "${candump_log}"
            ;;
        *)
            echo "unknown single-case name: ${name}" >&2
            exit 1
            ;;
    esac

    echo "[PASS] ${name}"
}

run_triple_case() {
    local candump_log="${LOG_DIR}/triple-candump.log"
    local qemu_log="${LOG_DIR}/triple-qemu.log"

    cleanup
    candump -L vcan0 vcan1 vcan2 > "${candump_log}" 2>&1 &
    CANDUMP_PID=$!
    sleep 1

    "${QEMU_BIN}" \
        -display none \
        -semihosting-config enable=on,target=native \
        -machine s32k1xx-can,canbus0=canbus0,canbus1=canbus1,canbus2=canbus2 \
        -object can-bus,id=canbus0 \
        -object can-bus,id=canbus1 \
        -object can-bus,id=canbus2 \
        -object can-host-socketcan,id=hostcan0,if=vcan0,canbus=canbus0 \
        -object can-host-socketcan,id=hostcan1,if=vcan1,canbus=canbus1 \
        -object can-host-socketcan,id=hostcan2,if=vcan2,canbus=canbus2 \
        -kernel "${TRIPLE_ELF}" > "${qemu_log}" 2>&1 &
    QEMU_PID=$!

    sleep 2
    cansend vcan0 121#1122334455667788
    sleep 1
    cansend vcan0 221##4101112131415161718191A1B
    sleep 1
    cansend vcan1 122#0102030405060708
    sleep 1
    cansend vcan1 222##4202122232425262728292A2B
    sleep 1
    cansend vcan2 123#A1A2A3A4A5A6A7A8
    sleep 1
    cansend vcan2 223##4303132333435363738393A3B
    sleep 2
    cleanup

    require_pattern "TRIPLE_PASS" "${qemu_log}"
    require_pattern "vcan0 304##5C0C1C2C3C4C5C6C7C8C9CACB" "${candump_log}"
    require_pattern "vcan1 404##5C0C1C2C3C4C5C6C7C8C9CACB" "${candump_log}"
    require_pattern "vcan2 504##5C0C1C2C3C4C5C6C7C8C9CACB" "${candump_log}"

    echo "[PASS] triple"
}

run_official_suite() {
    run_single_case \
        "official-flexcan" \
        "${CLASSIC_ELF}" \
        "cansend vcan0 511#0011223344556677"

    run_single_case \
        "official-canfd" \
        "${CANFD_ELF}" \
        "cansend vcan0 511##50011223344556677"

    run_single_case \
        "official-rxfifo" \
        "${RXFIFO_ELF}" \
        "cansend vcan0 511#0011223344556677; sleep 1; cansend vcan0 511#8899AABBCCDDEEFF; sleep 1; cansend vcan0 511#1021324354657687"
}

main() {
    require_file "${QEMU_BIN}"
    require_file "${CLASSIC_ELF}"
    require_file "${CANFD_ELF}"
    require_file "${RXFIFO_ELF}"
    require_file "${TRIPLE_ELF}"
    setup_vcan

    case "${MODE}" in
        official)
            run_official_suite
            ;;
        triple)
            run_triple_case
            ;;
        all)
            run_official_suite
            run_triple_case
            ;;
        *)
            echo "usage: $0 [official|triple|all]" >&2
            exit 1
            ;;
    esac

    echo "logs: ${LOG_DIR}"
}

main "$@"
