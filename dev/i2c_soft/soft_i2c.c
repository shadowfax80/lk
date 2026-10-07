/*
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */

/*
 * Software-simulated I2C bus with an attached EEPROM device.
 *
 * Targets such as qemu-virt-riscv have no I2C controller hardware, so this
 * module implements LK's I2C API (see <dev/i2c.h>) on top of a software model
 * of the bus: START/STOP/repeated-START framing, 7-bit addressing with
 * ACK/NACK, and a 24C02-like 256 byte EEPROM slave at address 0x50. It
 * provides bus 0 through strong definitions of i2c_*, which take precedence
 * over the weak aliases dev/gpio_i2c installs (that module is in some builds
 * but has no buses on targets without GPIO hardware), so ordinary I2C client
 * code works unchanged.
 *
 * Shell commands:
 *   i2c_scan                        probe bus 0 for devices
 *   i2c_write <dev> <reg> <b0> ...   write hex bytes (max 16)
 *   i2c_read <dev> <reg> <len>        read hex bytes
 *   i2c_trace <on|off>              toggle the protocol trace
 *   i2c_selftest                    write/read/verify roundtrip test
 */

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <lk/console_cmd.h>
#include <lk/err.h>
#include <dev/i2c.h>

#define SOFT_I2C_BUS 0
#define EEPROM_ADDR 0x50
#define EEPROM_SIZE 256

/* Set false to silence the per-transaction trace (the scan and selftest
 * commands do this while they run). */
static bool trace_on = true;

/* --- simulated EEPROM slave (24C02-like) --- */

typedef enum {
    S_IDLE, /* between transactions */
    S_ADDR, /* waiting for the address byte after (re)START */
    S_WORD, /* write mode: waiting for the word-address byte */
    S_DATA, /* write mode: storing data bytes, auto-incrementing */
    S_READ, /* read mode: serving data bytes, auto-incrementing */
} slave_state_t;

static slave_state_t slave_state = S_IDLE;
static uint8_t eeprom_mem[EEPROM_SIZE];
static uint8_t eeprom_ptr;
static bool eeprom_inited;

static void soft_i2c_eeprom_init(void) {
    if (eeprom_inited) {
        return;
    }
    /* erased EEPROM reads back 0xff */
    memset(eeprom_mem, 0xff, sizeof(eeprom_mem));
    eeprom_ptr = 0;
    eeprom_inited = true;
}

static void slave_start(void) {
    slave_state = S_ADDR;
}

static void slave_stop(void) {
    slave_state = S_IDLE;
}

/* Master sends a byte to the slave. Returns true if the slave ACKs it. */
static bool slave_put(uint8_t byte) {
    switch (slave_state) {
        case S_ADDR: {
            uint8_t addr = (uint8_t)((unsigned int)byte >> 1);
            bool read = (byte & 1) != 0;
            if (addr != EEPROM_ADDR) {
                slave_state = S_IDLE;
                return false;
            }
            slave_state = read ? S_READ : S_WORD;
            return true;
        }
        case S_WORD:
            eeprom_ptr = byte;
            slave_state = S_DATA;
            return true;
        case S_DATA:
            eeprom_mem[eeprom_ptr++] = byte; /* uint8_t wraps 0xff -> 0x00 */
            return true;
        default:
            return false;
    }
}

/* Master reads a byte from the slave. Only meaningful in S_READ. */
static uint8_t slave_get(void) {
    if (slave_state != S_READ) {
        return 0xff;
    }
    return eeprom_mem[eeprom_ptr++];
}

/* --- master-side transfer helpers (with protocol trace) --- */

static bool t_send_addr(uint8_t address, bool read) {
    uint8_t b = (uint8_t)(((unsigned int)address << 1) | (read ? 1u : 0u));
    bool ack = slave_put(b);
    if (trace_on) {
        printf(" %02x%c(%s)", (unsigned int)b, read ? 'R' : 'W', ack ? "A" : "N");
    }
    return ack;
}

static bool t_send_byte(uint8_t b) {
    bool ack = slave_put(b);
    if (trace_on) {
        printf(" %02x(%s)", (unsigned int)b, ack ? "A" : "N");
    }
    return ack;
}

/* last == true means the master NACKs to end the read, per the spec. */
static uint8_t t_recv_byte(bool last) {
    uint8_t b = slave_get();
    if (trace_on) {
        printf(" %02x(%s)", (unsigned int)b, last ? "n" : "a");
    }
    return b;
}

/* --- LK facing I2C API (same semantics as dev/gpio_i2c) --- */

void soft_i2c_init_early(void) {
    soft_i2c_eeprom_init();
}

void soft_i2c_init(void) {
    soft_i2c_eeprom_init();
}

status_t soft_i2c_transmit(int bus, uint8_t address, const void *vbuf, size_t cnt) {
    soft_i2c_eeprom_init();
    if (bus != SOFT_I2C_BUS) {
        return ERR_NOT_FOUND;
    }
    DEBUG_ASSERT(vbuf || !cnt);
    const uint8_t *buf = (const uint8_t *)vbuf;

    if (trace_on) {
        printf("[i2c0] S");
    }
    slave_start();
    if (!t_send_addr(address, false)) {
        slave_stop();
        if (trace_on) {
            printf(" P NACK\n");
        }
        return ERR_I2C_NACK;
    }
    for (size_t j = 0; j < cnt; j++) {
        if (!t_send_byte(buf[j])) {
            slave_stop();
            if (trace_on) {
                printf(" P NACK\n");
            }
            return ERR_I2C_NACK;
        }
    }
    slave_stop();
    if (trace_on) {
        printf(" P OK\n");
    }
    return NO_ERROR;
}

status_t soft_i2c_receive(int bus, uint8_t address, void *vbuf, size_t cnt) {
    soft_i2c_eeprom_init();
    if (bus != SOFT_I2C_BUS) {
        return ERR_NOT_FOUND;
    }
    DEBUG_ASSERT(vbuf || !cnt);
    uint8_t *buf = (uint8_t *)vbuf;

    if (trace_on) {
        printf("[i2c0] S");
    }
    slave_start();
    if (!t_send_addr(address, true)) {
        slave_stop();
        if (trace_on) {
            printf(" P NACK\n");
        }
        return ERR_I2C_NACK;
    }
    for (size_t j = 0; j < cnt; j++) {
        buf[j] = t_recv_byte(j == cnt - 1);
    }
    slave_stop();
    if (trace_on) {
        printf(" P OK\n");
    }
    return NO_ERROR;
}

status_t soft_i2c_write_reg_bytes(int bus, uint8_t address, uint8_t reg,
                                  const uint8_t *buf, size_t cnt) {
    soft_i2c_eeprom_init();
    if (bus != SOFT_I2C_BUS) {
        return ERR_NOT_FOUND;
    }
    DEBUG_ASSERT(buf || !cnt);

    if (trace_on) {
        printf("[i2c0] S");
    }
    slave_start();
    if (!t_send_addr(address, false) || !t_send_byte(reg)) {
        slave_stop();
        if (trace_on) {
            printf(" P NACK\n");
        }
        return ERR_I2C_NACK;
    }
    for (size_t j = 0; j < cnt; j++) {
        if (!t_send_byte(buf[j])) {
            slave_stop();
            if (trace_on) {
                printf(" P NACK\n");
            }
            return ERR_I2C_NACK;
        }
    }
    slave_stop();
    if (trace_on) {
        printf(" P OK\n");
    }
    return NO_ERROR;
}

status_t soft_i2c_read_reg_bytes(int bus, uint8_t address, uint8_t reg,
                                 uint8_t *buf, size_t cnt) {
    soft_i2c_eeprom_init();
    if (bus != SOFT_I2C_BUS) {
        return ERR_NOT_FOUND;
    }
    DEBUG_ASSERT(buf || !cnt);

    if (trace_on) {
        printf("[i2c0] S");
    }
    slave_start();
    if (!t_send_addr(address, false) || !t_send_byte(reg)) {
        slave_stop();
        if (trace_on) {
            printf(" P NACK\n");
        }
        return ERR_I2C_NACK;
    }
    if (trace_on) {
        printf(" Rs");
    }
    slave_start(); /* repeated START */
    if (!t_send_addr(address, true)) {
        slave_stop();
        if (trace_on) {
            printf(" P NACK\n");
        }
        return ERR_I2C_NACK;
    }
    for (size_t j = 0; j < cnt; j++) {
        buf[j] = t_recv_byte(j == cnt - 1);
    }
    slave_stop();
    if (trace_on) {
        printf(" P OK\n");
    }
    return NO_ERROR;
}

/* Strong definitions of LK's I2C API: these win over dev/gpio_i2c's weak
 * aliases wherever both modules are in the build, so bus 0 is the soft bus. */
void i2c_init_early(void) {
    soft_i2c_init_early();
}

void i2c_init(void) {
    soft_i2c_init();
}

status_t i2c_transmit(int bus, uint8_t address, const void *buf, size_t count) {
    return soft_i2c_transmit(bus, address, buf, count);
}

status_t i2c_receive(int bus, uint8_t address, void *buf, size_t count) {
    return soft_i2c_receive(bus, address, buf, count);
}

status_t i2c_write_reg_bytes(int bus, uint8_t address, uint8_t reg, const uint8_t *val, size_t cnt) {
    return soft_i2c_write_reg_bytes(bus, address, reg, val, cnt);
}

status_t i2c_read_reg_bytes(int bus, uint8_t address, uint8_t reg, uint8_t *val, size_t cnt) {
    return soft_i2c_read_reg_bytes(bus, address, reg, val, cnt);
}

/* --- shell commands --- */

static bool parse_hex8(const char *s, uint8_t *out) {
    char *end;
    unsigned long v;

    if (s == NULL || *s == '\0') {
        return false;
    }
    v = strtoul(s, &end, 16);
    if (*end != '\0' || v > 0xff) {
        return false;
    }
    *out = (uint8_t)v;
    return true;
}

static const char *i2c_status_str(status_t s) {
    if (s == NO_ERROR) {
        return "OK";
    }
    if (s == ERR_I2C_NACK) {
        return "NACK (no device?)";
    }
    if (s == ERR_NOT_FOUND) {
        return "bad bus";
    }
    return "error";
}

static int cmd_i2c_scan(int argc, const console_cmd_args *argv) {
    bool saved_trace = trace_on;
    int found = 0;

    trace_on = false; /* keep the 112 probes quiet */
    printf("scanning i2c bus 0 (0x08-0x77):\n");
    for (unsigned int a = 0x08; a <= 0x77; a++) {
        if (i2c_transmit(SOFT_I2C_BUS, (uint8_t)a, NULL, 0) == NO_ERROR) {
            printf("  device ACK at 0x%02x\n", a);
            found++;
        }
    }
    trace_on = saved_trace;
    printf("scan done: %d device(s)\n", found);
    return 0;
}

static int cmd_i2c_write(int argc, const console_cmd_args *argv) {
    uint8_t dev, reg;
    uint8_t data[16];
    size_t n;

    if (argc < 4 || argc > 4 + (int)sizeof(data)) {
        printf("usage: i2c_write <dev7> <reg> <byte>... (hex, max 16 bytes)\n");
        return -1;
    }
    if (!parse_hex8(argv[1].str, &dev) || !parse_hex8(argv[2].str, &reg)) {
        printf("bad dev/reg: want hex bytes\n");
        return -1;
    }
    n = (size_t)(argc - 3);
    for (int k = 0; k < argc - 3; k++) {
        if (!parse_hex8(argv[3 + k].str, &data[k])) {
            printf("bad data byte #%d: want hex byte\n", k);
            return -1;
        }
    }
    status_t st = i2c_write_reg_bytes(SOFT_I2C_BUS, dev, reg, data, n);
    printf("write %u byte(s) to dev 0x%02x reg 0x%02x: %s (%d)\n",
           (unsigned int)n, (unsigned int)dev, (unsigned int)reg,
           i2c_status_str(st), (int)st);
    return (st == NO_ERROR) ? 0 : -1;
}

static int cmd_i2c_read(int argc, const console_cmd_args *argv) {
    uint8_t dev, reg, len;
    uint8_t data[256];

    if (argc != 4) {
        printf("usage: i2c_read <dev7> <reg> <len> (hex)\n");
        return -1;
    }
    if (!parse_hex8(argv[1].str, &dev) || !parse_hex8(argv[2].str, &reg) ||
            !parse_hex8(argv[3].str, &len) || len == 0) {
        printf("bad args: want hex dev, reg and nonzero len\n");
        return -1;
    }
    status_t st = i2c_read_reg_bytes(SOFT_I2C_BUS, dev, reg, data, len);
    if (st != NO_ERROR) {
        printf("read from dev 0x%02x reg 0x%02x: %s (%d)\n",
               (unsigned int)dev, (unsigned int)reg, i2c_status_str(st), (int)st);
        return -1;
    }
    printf("dev 0x%02x reg 0x%02x:", (unsigned int)dev, (unsigned int)reg);
    for (unsigned int k = 0; k < len; k++) {
        printf(" %02x", (unsigned int)data[k]);
    }
    printf("\n");
    return 0;
}

static int cmd_i2c_trace(int argc, const console_cmd_args *argv) {
    if (argc == 2 && !strcmp(argv[1].str, "on")) {
        trace_on = true;
    } else if (argc == 2 && !strcmp(argv[1].str, "off")) {
        trace_on = false;
    } else {
        printf("usage: i2c_trace <on|off> (currently %s)\n", trace_on ? "on" : "off");
        return -1;
    }
    printf("i2c trace %s\n", trace_on ? "on" : "off");
    return 0;
}

static int cmd_i2c_selftest(int argc, const console_cmd_args *argv) {
    bool saved_trace = trace_on;
    int failures = 0;
    status_t st;

    trace_on = false;
    printf("i2c selftest: bus 0, eeprom at 0x50\n");

    /* 4-byte write/read roundtrip */
    const uint8_t wpat[4] = { 0xde, 0xad, 0xbe, 0xef };
    uint8_t rbuf[4] = { 0, 0, 0, 0 };
    st = i2c_write_reg_bytes(SOFT_I2C_BUS, EEPROM_ADDR, 0x10, wpat, sizeof(wpat));
    if (st != NO_ERROR) {
        printf("FAIL: 4-byte write -> %d\n", (int)st);
        failures++;
    } else {
        st = i2c_read_reg_bytes(SOFT_I2C_BUS, EEPROM_ADDR, 0x10, rbuf, sizeof(rbuf));
        if (st != NO_ERROR) {
            printf("FAIL: 4-byte read -> %d\n", (int)st);
            failures++;
        } else if (memcmp(wpat, rbuf, sizeof(wpat)) != 0) {
            printf("FAIL: readback %02x %02x %02x %02x, want de ad be ef\n",
                   (unsigned int)rbuf[0], (unsigned int)rbuf[1],
                   (unsigned int)rbuf[2], (unsigned int)rbuf[3]);
            failures++;
        } else {
            printf("PASS: 4-byte write/read roundtrip\n");
        }
    }

    /* single-byte write/read */
    st = i2c_write_reg(SOFT_I2C_BUS, EEPROM_ADDR, 0x20, 0xa5);
    if (st != NO_ERROR) {
        printf("FAIL: 1-byte write -> %d\n", (int)st);
        failures++;
    } else {
        uint8_t v = 0;
        st = i2c_read_reg(SOFT_I2C_BUS, EEPROM_ADDR, 0x20, &v);
        if (st != NO_ERROR || v != 0xa5) {
            printf("FAIL: 1-byte readback 0x%02x (-> %d), want 0xa5\n",
                   (unsigned int)v, (int)st);
            failures++;
        } else {
            printf("PASS: single-byte write/read\n");
        }
    }

    /* a missing device must NACK */
    uint8_t dummy = 0;
    st = i2c_read_reg_bytes(SOFT_I2C_BUS, EEPROM_ADDR + 1, 0x00, &dummy, 1);
    if (st == ERR_I2C_NACK) {
        printf("PASS: missing device 0x51 NACKs\n");
    } else {
        printf("FAIL: read from 0x51 -> %d, want NACK\n", (int)st);
        failures++;
    }

    /* address wraparound: 2 bytes at 0xff spill over to 0x00 */
    const uint8_t wwrap[2] = { 0x11, 0x22 };
    uint8_t rwrap[2] = { 0, 0 };
    st = i2c_write_reg_bytes(SOFT_I2C_BUS, EEPROM_ADDR, 0xff, wwrap, sizeof(wwrap));
    if (st != NO_ERROR) {
        printf("FAIL: wrap write -> %d\n", (int)st);
        failures++;
    } else {
        st = i2c_read_reg_bytes(SOFT_I2C_BUS, EEPROM_ADDR, 0xff, rwrap, sizeof(rwrap));
        if (st != NO_ERROR || memcmp(wwrap, rwrap, sizeof(wwrap)) != 0) {
            printf("FAIL: wrap readback %02x %02x, want 11 22\n",
                   (unsigned int)rwrap[0], (unsigned int)rwrap[1]);
            failures++;
        } else {
            printf("PASS: address wraparound\n");
        }
    }

    trace_on = saved_trace;
    printf("i2c selftest: %s (%d failure(s))\n", failures == 0 ? "PASS" : "FAIL", failures);
    return (failures == 0) ? 0 : -1;
}

STATIC_COMMAND_START
STATIC_COMMAND("i2c_scan", "scan i2c bus 0 for devices", &cmd_i2c_scan)
STATIC_COMMAND("i2c_write", "write i2c eeprom bytes", &cmd_i2c_write)
STATIC_COMMAND("i2c_read", "read i2c eeprom bytes", &cmd_i2c_read)
STATIC_COMMAND("i2c_trace", "i2c protocol trace on/off", &cmd_i2c_trace)
STATIC_COMMAND("i2c_selftest", "i2c eeprom selftest", &cmd_i2c_selftest)
STATIC_COMMAND_END(i2c_soft);
