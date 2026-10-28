#include "motor_test.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint8_t packets[256][16];
static size_t lengths[256], count;
static bool fail_next;
static size_t fail_at = SIZE_MAX;
static char logs[8192];
bool platform_motor_send(const uint8_t *data, size_t len)
{
    assert(count < 256 && len <= 16);
    assert(data[0] >= 1 && data[0] <= 4); /* 任意路径都不能碰地址 0/5。 */
    memcpy(packets[count], data, len); lengths[count++] = len;
    if (fail_next || count - 1U == fail_at) { fail_next = false; return false; }
    return true;
}
void platform_log(const char *text)
{
    size_t a = strlen(logs), b = strlen(text);
    assert(a+b < sizeof logs);
    memcpy(logs+a, text, b+1);
}
static size_t moves(void)
{
    size_t n = 0;
    for (size_t i=0; i<count; ++i) if (packets[i][1] == 0xFD) ++n;
    return n;
}
static void reset(void) { count = 0; fail_next = false; fail_at = SIZE_MAX; logs[0] = 0; test_init(); }
static void receive(const uint8_t *data, size_t len, uint32_t now)
{
    for (size_t i=0; i<len; ++i) test_motor_rx(data[i], now);
}

static void check_group_pattern(const uint8_t expected_directions[5])
{
    bool seen[5] = {false};
    size_t n = 0;
    for (size_t i=0; i<count; ++i) {
        if (packets[i][1] != 0xFD) continue;
        uint8_t id = packets[i][0];
        assert(!seen[id]); seen[id] = true; ++n;
        const uint8_t expected[] = {id,0xFD,expected_directions[id],0,5,10,0,0,0,160,0,0,0x6B};
        assert(lengths[i] == sizeof expected && !memcmp(packets[i],expected,sizeof expected));
    }
    assert(n == 4 && seen[1] && seen[2] && seen[3] && seen[4]);
}

static void check_group_moves(uint8_t expected_direction)
{
    const uint8_t expected[5] = {0, expected_direction, expected_direction,
                               expected_direction, expected_direction};
    check_group_pattern(expected);
}

static void check_long_pattern(const uint8_t directions[5])
{
    bool seen[5] = {false};
    for (size_t i=0; i<count; ++i) {
        if (packets[i][1] != 0xFD) continue;
        uint8_t id = packets[i][0];
        const uint8_t expected[] = {id,0xFD,directions[id],0,5,10,0,0,6,0x40,0,0,0x6B};
        assert(!seen[id]); seen[id] = true;
        assert(lengths[i] == sizeof expected && !memcmp(packets[i],expected,sizeof expected));
    }
    assert(moves() == 4 && seen[1] && seen[2] && seen[3] && seen[4]);
}

static void check_long_left_frames(void)
{
    const uint8_t directions[5] = {0,1,0,1,0};
    check_long_pattern(directions);
}

static void test_long_left(void)
{
    reset(); test_command("left_long",0);test_tick(100);
    assert(count == 8 && moves() == 0);
    reset(); test_command("arm 3",0);test_command("left_long",1);
    test_command("jog 1",2);test_tick(100);assert(count == 8);
    reset(); test_command("arm all",0);test_command("left_long",10000);
    test_tick(10050);assert(count == 8 && moves() == 0);

    reset(); test_command("arm all",0);test_command("left_long",1);
    test_tick(50);assert(count == 12 && moves() == 0);
    test_tick(51);check_long_left_frames();assert(count == 16);
    test_command("left_long",52);assert(count == 16); /* 单次授权不能重发。 */
    test_tick(3001);assert(count == 16); /* 仅长左横移允许超过 3 秒。 */
    test_tick(8000);assert(count == 16);
    test_tick(8001);assert(count == 24);
    assert(strstr(logs,"STOP/DISABLE sent after 8 seconds (left_long)"));
    for (uint8_t id=1; id<=4; ++id) {
        size_t at = 16 + 2U * (id-1U);
        const uint8_t stop[] = {id,0xFE,0x98,0,0x6B};
        const uint8_t disable[] = {id,0xF3,0xAB,0,0,0x6B};
        assert(lengths[at] == sizeof stop && !memcmp(packets[at],stop,sizeof stop));
        assert(lengths[at+1] == sizeof disable && !memcmp(packets[at+1],disable,sizeof disable));
    }

    reset(); test_command("arm all",0);test_command("left_long",1);
    test_command("stop",51);test_tick(1000);assert(count == 20 && moves() == 0);
    reset(); test_command("arm all",0);test_command("left_long",1);
    test_tick(8001);assert(count == 20 && moves() == 0); /* 调度迟到不得补发。 */
    reset(); test_command("arm all",0);test_command("probe 3",1);
    test_command("left_long",2);test_tick(100);assert(count == 9 && moves() == 0);
    reset(); test_command("arm all",0);test_command("left_long extra",1);
    test_command("left_long",2);test_tick(100);assert(count == 8 && moves() == 0);

    reset(); test_command("arm all",0);test_command("left_long",1);
    test_fault();test_tick(10000);assert(count == 20 && moves() == 0);
    reset(); test_command("arm all",0);fail_at=9;test_command("left_long",1);
    test_tick(100);assert(count == 18 && moves() == 0);
    reset(); test_command("arm all",0);test_command("left_long",1);
    fail_at=13;test_tick(51);assert(count == 22 && moves() == 2);
    test_tick(10000);assert(count == 22);

    reset(); test_command("arm all",UINT32_MAX-100);
    test_command("left_long",UINT32_MAX-50);test_tick(0);check_long_left_frames();
    test_tick(7948);assert(count == 16);test_tick(7949);assert(count == 24);

    /* 长动作后不复位：后续普通左横移和单轮必须恢复 160 脉冲/3 秒。 */
    reset(); test_command("arm all",0);test_command("left_long",1);
    test_tick(51);test_command("stop",100);assert(count == 24);
    count=0; logs[0]=0;
    test_command("arm all",101);test_command("left",102);test_tick(152);
    const uint8_t short_left[5] = {0,1,0,1,0};
    check_group_pattern(short_left);assert(count == 8);
    test_tick(3101);assert(count == 8);test_tick(3102);assert(count == 16);
    assert(strstr(logs,"STOP/DISABLE sent after 3 seconds."));
    count=0; logs[0]=0;
    test_command("arm 3",3200);test_command("jog 1",3201);test_tick(3251);
    const uint8_t single[] = {3,0xFD,1,0,5,10,0,0,0,160,0,0,0x6B};
    assert(count == 2 && lengths[1] == sizeof single && !memcmp(packets[1],single,sizeof single));
    test_tick(6201);assert(count == 10);
}

static void check_long_right_frames(void)
{
    const uint8_t directions[5] = {0,0,1,0,1};
    check_long_pattern(directions);
}

static void test_long_right(void)
{
    reset(); test_command("right_long",0);test_tick(100);
    assert(count == 8 && moves() == 0);
    reset(); test_command("arm 3",0);test_command("right_long",1);
    test_command("jog 1",2);test_tick(100);assert(count == 8);
    reset(); test_command("arm all",0);test_command("right_long",10000);
    test_tick(10050);assert(count == 8 && moves() == 0);

    reset(); test_command("arm all",0);test_command("right_long",1);
    test_tick(50);assert(count == 12 && moves() == 0);
    test_tick(51);check_long_right_frames();assert(count == 16);
    test_command("right_long",52);assert(count == 16);
    test_tick(3001);assert(count == 16);
    test_tick(8000);assert(count == 16);
    test_tick(8001);assert(count == 24);
    assert(strstr(logs,"STOP/DISABLE sent after 8 seconds (right_long)"));
    for (uint8_t id=1; id<=4; ++id) {
        size_t at = 16 + 2U * (id-1U);
        const uint8_t stop[] = {id,0xFE,0x98,0,0x6B};
        const uint8_t disable[] = {id,0xF3,0xAB,0,0,0x6B};
        assert(lengths[at] == sizeof stop && !memcmp(packets[at],stop,sizeof stop));
        assert(lengths[at+1] == sizeof disable && !memcmp(packets[at+1],disable,sizeof disable));
    }

    reset(); test_command("arm all",0);test_command("right_long",1);
    test_command("stop",51);test_tick(1000);assert(count == 20 && moves() == 0);
    reset(); test_command("arm all",0);test_command("right_long",1);
    test_tick(8001);assert(count == 20 && moves() == 0);
    reset(); test_command("arm all",0);test_command("probe 3",1);
    test_command("right_long",2);test_tick(100);assert(count == 9 && moves() == 0);
    reset(); test_command("arm all",0);test_command("right_long extra",1);
    test_command("right_long",2);test_tick(100);assert(count == 8 && moves() == 0);
    reset(); test_command("arm all",0);test_command("right_long",1);
    test_fault();test_tick(10000);assert(count == 20 && moves() == 0);
    reset(); test_command("arm all",0);fail_at=9;test_command("right_long",1);
    test_tick(100);assert(count == 18 && moves() == 0);
    reset(); test_command("arm all",0);test_command("right_long",1);
    fail_at=13;test_tick(51);assert(count == 22 && moves() == 2);
    test_tick(10000);assert(count == 22);

    reset(); test_command("arm all",UINT32_MAX-100);
    test_command("right_long",UINT32_MAX-50);test_tick(0);check_long_right_frames();
    test_tick(7948);assert(count == 16);test_tick(7949);assert(count == 24);

    /* 不复位依次右长、左长、右短、单轮，核对方向/行程/时限不串用。 */
    reset(); test_command("arm all",0);test_command("right_long",1);
    test_tick(51);test_command("stop",100);
    count=0;logs[0]=0;
    test_command("arm all",101);test_command("left_long",102);test_tick(152);
    check_long_left_frames();assert(count == 8);
    assert(strstr(logs,"LONG JOG sent: left_long;") && !strstr(logs,"LONG JOG sent: right_long;"));
    test_tick(8102);assert(count == 16);
    assert(strstr(logs,"STOP/DISABLE sent after 8 seconds (left_long)"));
    count=0;logs[0]=0;
    test_command("arm all",8200);test_command("right",8201);test_tick(8251);
    const uint8_t short_right[5] = {0,0,1,0,1};
    check_group_pattern(short_right);assert(count == 8);
    test_tick(11200);assert(count == 8);test_tick(11201);assert(count == 16);
    assert(strstr(logs,"STOP/DISABLE sent after 3 seconds."));
    count=0;logs[0]=0;
    test_command("arm 3",11300);test_command("jog 1",11301);test_tick(11351);
    const uint8_t single[] = {3,0xFD,1,0,5,10,0,0,0,160,0,0,0x6B};
    assert(count == 2 && lengths[1] == sizeof single && !memcmp(packets[1],single,sizeof single));
    test_tick(14301);assert(count == 10);
}

static void test_six_long_commands(void)
{
    /* 预期地址方向来自实物轮号与车体运动定义，未调用被测方向函数。 */
    static const struct {
        const char *long_command, *short_command;
        uint8_t directions[5];
    } cases[] = {
        {"forward_long", "forward", {0,1,1,1,1}},
        {"back_long", "back",       {0,0,0,0,0}},
        {"left_long", "left",       {0,1,0,1,0}},
        {"right_long", "right",     {0,0,1,0,1}},
        {"cw_long", "cw",           {0,0,0,1,1}},
        {"ccw_long", "ccw",         {0,1,1,0,0}}
    };
    for (size_t c=0; c<sizeof cases/sizeof cases[0]; ++c) {
        const char *cmd=cases[c].long_command;
        reset();test_command(cmd,0);test_tick(100);
        assert(count==8 && moves()==0);
        reset();test_command("arm 3",0);test_command(cmd,1);
        test_command("jog 1",2);test_tick(100);assert(count==8 && moves()==0);
        reset();test_command("arm all",0);test_command(cmd,10000);
        test_tick(10050);assert(count==8 && moves()==0);

        reset();test_command("arm all",0);test_command(cmd,1);
        test_tick(50);assert(count==12 && moves()==0);
        test_tick(51);check_long_pattern(cases[c].directions);assert(count==16);
        test_command(cmd,52);test_command("arm all",53);assert(count==16);
        test_tick(3001);assert(count==16);test_tick(8000);assert(count==16);
        test_tick(8001);assert(count==24);
        char expected_log[128];
        snprintf(expected_log,sizeof expected_log,"STOP/DISABLE sent after 8 seconds (%s).",cmd);
        assert(strstr(logs,expected_log));
        for (uint8_t id=1;id<=4;++id) {
            size_t at=16+2U*(id-1U);
            const uint8_t stop[]={id,0xFE,0x98,0,0x6B};
            const uint8_t disable[]={id,0xF3,0xAB,0,0,0x6B};
            assert(lengths[at]==sizeof stop && !memcmp(packets[at],stop,sizeof stop));
            assert(lengths[at+1]==sizeof disable && !memcmp(packets[at+1],disable,sizeof disable));
        }
        /* 同一会话接短动作，不能遗留长行程、长时限或长动作名称。 */
        count=0;logs[0]=0;
        test_command("arm all",8100);test_command(cases[c].short_command,8101);
        test_tick(8151);check_group_pattern(cases[c].directions);assert(count==8);
        test_tick(11100);assert(count==8);test_tick(11101);assert(count==16);
        assert(strstr(logs,"STOP/DISABLE sent after 3 seconds."));
        assert(!strstr(logs,"LONG JOG sent:"));

        reset();test_command("arm all",0);test_command(cmd,1);
        test_command("stop",51);test_tick(1000);assert(count==20 && moves()==0);
        reset();test_command("arm all",0);test_command(cmd,1);
        test_tick(8001);assert(count==20 && moves()==0);
        reset();test_command("arm all",0);test_command("probe 3",1);
        test_command(cmd,2);test_tick(100);assert(count==9 && moves()==0);
        reset();test_command("arm all",0);test_command(cmd,1);
        test_tick(51);test_fault();test_tick(10000);assert(count==24 && moves()==4);
        test_command(cmd,10001);assert(count==24);
        reset();test_command("arm all",0);fail_at=9;test_command(cmd,1);
        test_tick(100);assert(count==18 && moves()==0);
        reset();test_command("arm all",0);test_command(cmd,1);
        fail_at=13;test_tick(51);assert(count==22 && moves()==2);
        test_tick(10000);assert(count==22);
        reset();test_command("arm all",UINT32_MAX-100);
        test_command(cmd,UINT32_MAX-50);test_tick(0);check_long_pattern(cases[c].directions);
        test_tick(7948);assert(count==16);test_tick(7949);assert(count==24);
        char invalid[32];snprintf(invalid,sizeof invalid,"%s extra",cmd);
        reset();test_command("arm all",0);test_command(invalid,1);
        test_command(cmd,2);test_tick(100);assert(count==8 && moves()==0);
    }
    reset();test_command("arm all",0);test_command("*_long",1);
    test_command("ccw_long",2);test_tick(100);assert(count==8 && moves()==0);
}

static void check_calibration_forward_frames(void)
{
    bool seen[5] = {false};
    for (size_t i=0; i<count; ++i) {
        if (packets[i][1] != 0xFD) continue;
        uint8_t id = packets[i][0];
        /* 实际协议中的 16000=0x3E80，四轮方向均为 1。 */
        const uint8_t expected[] = {id,0xFD,1,0,5,10,0,0,0x3E,0x80,0,0,0x6B};
        assert(!seen[id]); seen[id] = true;
        assert(lengths[i] == sizeof expected && !memcmp(packets[i],expected,sizeof expected));
    }
    assert(moves() == 4 && seen[1] && seen[2] && seen[3] && seen[4]);
}

static void test_calibration_forward(void)
{
    reset();test_command("forward_16000",0);test_tick(100);
    assert(count==8 && moves()==0);
    reset();test_command("arm 3",0);test_command("forward_16000",1);
    test_command("jog 1",2);test_tick(100);assert(count==8 && moves()==0);
    reset();test_command("arm all",0);test_command("forward_16000",10000);
    test_tick(10050);assert(count==8 && moves()==0);
    reset();test_command("arm all",0);test_command("jog 1",1);
    test_command("forward_16000",2);test_tick(100);assert(count==8 && moves()==0);

    reset();test_command("arm all",0);test_command("forward_16000",1);
    test_tick(50);assert(count==12 && moves()==0);
    test_tick(51);check_calibration_forward_frames();assert(count==16);
    assert(strstr(logs,"CALIBRATION JOG sent: forward_16000; 5 RPM, 16000 pulses, 80-second stop deadline."));
    test_command("forward_16000",52);test_command("arm all",53);
    assert(count==16); /* 不重发，也不能通过再次 arm 延长本次时限。 */
    test_tick(3001);test_tick(8001);test_tick(80000);assert(count==16);
    test_tick(80001);assert(count==24);
    assert(strstr(logs,"STOP/DISABLE sent after 80 seconds (forward_16000)"));
    for (uint8_t id=1;id<=4;++id) {
        size_t at=16+2U*(id-1U);
        const uint8_t stop[]={id,0xFE,0x98,0,0x6B};
        const uint8_t disable[]={id,0xF3,0xAB,0,0,0x6B};
        assert(lengths[at]==sizeof stop && !memcmp(packets[at],stop,sizeof stop));
        assert(lengths[at+1]==sizeof disable && !memcmp(packets[at+1],disable,sizeof disable));
    }
    test_tick(160001);test_command("forward_16000",160002);assert(count==24);

    /* 80 秒动作不能泄漏到原 8 秒或 3 秒动作。 */
    count=0;logs[0]=0;
    test_command("arm all",160100);test_command("left_long",160101);test_tick(160151);
    check_long_left_frames();assert(count==8);
    test_tick(168100);assert(count==8);test_tick(168101);assert(count==16);
    assert(strstr(logs,"STOP/DISABLE sent after 8 seconds (left_long)"));
    assert(!strstr(logs,"CALIBRATION JOG"));
    count=0;logs[0]=0;
    test_command("arm all",168200);test_command("forward",168201);test_tick(168251);
    check_group_moves(1);assert(count==8);
    test_tick(171200);assert(count==8);test_tick(171201);assert(count==16);
    assert(strstr(logs,"STOP/DISABLE sent after 3 seconds."));

    reset();test_command("arm all",0);test_command("forward_16000",1);
    test_command("stop",51);test_tick(80001);assert(count==20 && moves()==0);
    reset();test_command("arm all",0);test_command("forward_16000",1);
    test_tick(51);test_command("stop",1000);test_tick(80001);
    assert(count==24 && moves()==4 && !strstr(logs,"STOP/DISABLE sent after 80 seconds"));
    reset();test_command("arm all",0);test_command("forward_16000",1);
    test_tick(80001);assert(count==20 && moves()==0); /* 过期不补发。 */
    reset();test_command("arm all",0);test_command("probe 3",1);
    test_command("forward_16000",2);test_tick(100);assert(count==9 && moves()==0);
    reset();test_command("arm all",0);test_command("forward_16000",1);
    test_tick(51);test_fault();test_tick(80001);assert(count==24 && moves()==4);
    test_command("forward_16000",80002);assert(count==24);
    reset();test_command("arm all",0);fail_at=9;test_command("forward_16000",1);
    test_tick(100);assert(count==18 && moves()==0);
    reset();test_command("arm all",0);test_command("forward_16000",1);
    fail_at=13;test_tick(51);assert(count==22 && moves()==2);
    test_tick(80001);assert(count==22);
    reset();test_command("arm all",UINT32_MAX-100);
    test_command("forward_16000",UINT32_MAX-50);test_tick(0);check_calibration_forward_frames();
    test_tick(79948);assert(count==16);test_tick(79949);assert(count==24);

    const char *invalid[]={"forward_16000 extra","forward_160000","forward 16000","back_16000"};
    for (size_t i=0;i<sizeof invalid/sizeof invalid[0];++i) {
        reset();test_command("arm all",0);test_command(invalid[i],1);
        test_command("forward_16000",2);test_tick(100);assert(count==8 && moves()==0);
    }
}

static void check_forward_1m_frames(void)
{
    bool seen[5] = {false};
    for (size_t i=0;i<count;++i) {
        if (packets[i][1]!=0xFD) continue;
        uint8_t id=packets[i][0];
        /* 独立检查实际字节：12924=0x327C，四轮均向车头。 */
        const uint8_t expected[]={id,0xFD,1,0,5,10,0,0,0x32,0x7C,0,0,0x6B};
        assert(!seen[id]);seen[id]=true;
        assert(lengths[i]==sizeof expected && !memcmp(packets[i],expected,sizeof expected));
    }
    assert(moves()==4 && seen[1] && seen[2] && seen[3] && seen[4]);
}

static void test_forward_1m(void)
{
    reset();test_command("forward_1m",0);test_tick(100);assert(count==8 && moves()==0);
    reset();test_command("arm 3",0);test_command("forward_1m",1);
    test_command("jog 1",2);test_tick(100);assert(count==8 && moves()==0);
    reset();test_command("arm all",0);test_command("forward_1m",10000);
    test_tick(10050);assert(count==8 && moves()==0);
    reset();test_command("arm all",0);test_command("forward_1m",1);
    test_tick(50);assert(count==12 && moves()==0);
    test_tick(51);check_forward_1m_frames();assert(count==16);
    assert(strstr(logs,"VALIDATION JOG sent: forward_1m; nominal 100 cm, 5 RPM, 12924 pulses, 80-second stop deadline."));
    assert(!strstr(logs,"CALIBRATION JOG sent:"));
    test_command("forward_1m",52);test_command("arm all",53);assert(count==16);
    test_tick(3001);test_tick(8001);test_tick(80000);assert(count==16);
    test_tick(80001);assert(count==24);
    assert(strstr(logs,"STOP/DISABLE sent after 80 seconds (forward_1m)."));
    assert(!strstr(logs,"STOP/DISABLE sent after 80 seconds (forward_16000)."));
    for (uint8_t id=1;id<=4;++id) {
        size_t at=16+2U*(id-1U);
        const uint8_t stop[]={id,0xFE,0x98,0,0x6B};
        const uint8_t disable[]={id,0xF3,0xAB,0,0,0x6B};
        assert(lengths[at]==sizeof stop && !memcmp(packets[at],stop,sizeof stop));
        assert(lengths[at+1]==sizeof disable && !memcmp(packets[at+1],disable,sizeof disable));
    }
    test_tick(160001);test_command("forward_1m",160002);assert(count==24);

    /* 同会话依次换回 16000、1600、160 脉冲，行程、名称和期限不能串用。 */
    count=0;logs[0]=0;
    test_command("arm all",160100);test_command("forward_16000",160101);test_tick(160151);
    check_calibration_forward_frames();assert(count==8);
    test_tick(240100);assert(count==8);test_tick(240101);assert(count==16);
    assert(strstr(logs,"STOP/DISABLE sent after 80 seconds (forward_16000)."));
    count=0;logs[0]=0;
    test_command("arm all",240200);test_command("forward_long",240201);test_tick(240251);
    const uint8_t directions[]={0,1,1,1,1};check_long_pattern(directions);
    test_tick(248200);assert(count==8);test_tick(248201);assert(count==16);
    assert(strstr(logs,"STOP/DISABLE sent after 8 seconds (forward_long)."));
    count=0;logs[0]=0;
    test_command("arm all",248300);test_command("forward",248301);test_tick(248351);
    check_group_moves(1);test_tick(251300);assert(count==8);test_tick(251301);assert(count==16);
    assert(strstr(logs,"STOP/DISABLE sent after 3 seconds."));

    reset();test_command("arm all",0);test_command("forward_1m",1);
    test_command("stop",51);test_tick(80001);assert(count==20 && moves()==0);
    reset();test_command("arm all",0);test_command("forward_1m",1);
    test_tick(51);test_command("stop",1000);test_tick(80001);assert(count==24 && moves()==4);
    reset();test_command("arm all",0);test_command("forward_1m",1);
    test_tick(80001);assert(count==20 && moves()==0);
    reset();test_command("arm all",0);test_command("probe 3",1);
    test_command("forward_1m",2);test_tick(100);assert(count==9 && moves()==0);
    reset();test_command("arm all",0);test_command("forward_1m",1);
    test_tick(51);test_fault();test_tick(80001);assert(count==24 && moves()==4);
    test_command("forward_1m",80002);assert(count==24);
    reset();test_command("arm all",0);fail_at=9;test_command("forward_1m",1);
    test_tick(100);assert(count==18 && moves()==0);
    reset();test_command("arm all",0);test_command("forward_1m",1);
    fail_at=13;test_tick(51);assert(count==22 && moves()==2);test_tick(80001);assert(count==22);
    reset();test_command("arm all",UINT32_MAX-100);
    test_command("forward_1m",UINT32_MAX-50);test_tick(0);check_forward_1m_frames();
    test_tick(79948);assert(count==16);test_tick(79949);assert(count==24);
    const char *invalid[]={"forward_1m 2","forward_1m extra","forward_2m","back_1m extra"};
    for (size_t i=0;i<sizeof invalid/sizeof invalid[0];++i) {
        reset();test_command("arm all",0);test_command(invalid[i],1);
        test_command("forward_1m",2);test_tick(100);assert(count==8 && moves()==0);
    }
}

static void check_back_1m_frames(void)
{
    bool seen[5]={false};
    for (size_t i=0;i<count;++i) {
        if (packets[i][1]!=0xFD) continue;
        uint8_t id=packets[i][0];
        const uint8_t expected[]={id,0xFD,0,0,5,10,0,0,0x32,0x7C,0,0,0x6B};
        assert(!seen[id]);seen[id]=true;
        assert(lengths[i]==sizeof expected && !memcmp(packets[i],expected,sizeof expected));
    }
    assert(moves()==4 && seen[1] && seen[2] && seen[3] && seen[4]);
}

static void test_back_1m(void)
{
    reset();test_command("back_1m",0);test_tick(100);assert(count==8 && moves()==0);
    reset();test_command("arm 3",0);test_command("back_1m",1);
    test_command("jog 1",2);test_tick(100);assert(count==8 && moves()==0);
    reset();test_command("arm all",0);test_command("back_1m",10000);
    test_tick(10050);assert(count==8 && moves()==0);
    reset();test_command("arm all",0);test_command("back_1m",1);
    test_tick(50);assert(count==12 && moves()==0);
    test_tick(51);check_back_1m_frames();assert(count==16);
    assert(strstr(logs,"VALIDATION JOG sent: back_1m; nominal 100 cm, 5 RPM, 12924 pulses, 80-second stop deadline."));
    assert(!strstr(logs,"VALIDATION JOG sent: forward_1m;"));
    assert(!strstr(logs,"CALIBRATION JOG sent:"));
    test_command("back_1m",52);test_command("arm all",53);assert(count==16);
    test_tick(3001);test_tick(8001);test_tick(80000);assert(count==16);
    test_tick(80001);assert(count==24);
    assert(strstr(logs,"STOP/DISABLE sent after 80 seconds (back_1m)."));
    assert(!strstr(logs,"STOP/DISABLE sent after 80 seconds (forward_1m)."));
    for (uint8_t id=1;id<=4;++id) {
        size_t at=16+2U*(id-1U);
        const uint8_t stop[]={id,0xFE,0x98,0,0x6B};
        const uint8_t disable[]={id,0xF3,0xAB,0,0,0x6B};
        assert(lengths[at]==sizeof stop && !memcmp(packets[at],stop,sizeof stop));
        assert(lengths[at+1]==sizeof disable && !memcmp(packets[at+1],disable,sizeof disable));
    }
    test_tick(160001);test_command("back_1m",160002);assert(count==24);

    /* 先后退再前进，完整核对方向和动作名称，之后恢复 8 秒/3 秒短配置。 */
    count=0;logs[0]=0;
    test_command("arm all",160100);test_command("forward_1m",160101);test_tick(160151);
    check_forward_1m_frames();assert(count==8);
    test_tick(240100);assert(count==8);test_tick(240101);assert(count==16);
    assert(strstr(logs,"STOP/DISABLE sent after 80 seconds (forward_1m)."));
    assert(!strstr(logs,"VALIDATION JOG sent: back_1m;"));
    count=0;logs[0]=0;
    test_command("arm all",240200);test_command("back_long",240201);test_tick(240251);
    const uint8_t directions[]={0,0,0,0,0};check_long_pattern(directions);
    test_tick(248200);assert(count==8);test_tick(248201);assert(count==16);
    assert(strstr(logs,"STOP/DISABLE sent after 8 seconds (back_long)."));
    count=0;logs[0]=0;
    test_command("arm all",248300);test_command("back",248301);test_tick(248351);
    check_group_moves(0);test_tick(251300);assert(count==8);test_tick(251301);assert(count==16);
    assert(strstr(logs,"STOP/DISABLE sent after 3 seconds."));

    reset();test_command("arm all",0);test_command("back_1m",1);
    test_command("stop",51);test_tick(80001);assert(count==20 && moves()==0);
    reset();test_command("arm all",0);test_command("back_1m",1);
    test_tick(51);test_command("stop",1000);test_tick(80001);assert(count==24 && moves()==4);
    reset();test_command("arm all",0);test_command("back_1m",1);
    test_tick(80001);assert(count==20 && moves()==0);
    reset();test_command("arm all",0);test_command("probe 3",1);
    test_command("back_1m",2);test_tick(100);assert(count==9 && moves()==0);
    reset();test_command("arm all",0);test_command("back_1m",1);
    test_tick(51);test_fault();test_tick(80001);assert(count==24 && moves()==4);
    test_command("back_1m",80002);assert(count==24);
    reset();test_command("arm all",0);fail_at=9;test_command("back_1m",1);
    test_tick(100);assert(count==18 && moves()==0);
    reset();test_command("arm all",0);test_command("back_1m",1);
    fail_at=13;test_tick(51);assert(count==22 && moves()==2);test_tick(80001);assert(count==22);
    reset();test_command("arm all",UINT32_MAX-100);
    test_command("back_1m",UINT32_MAX-50);test_tick(0);check_back_1m_frames();
    test_tick(79948);assert(count==16);test_tick(79949);assert(count==24);
    const char *invalid[]={"back_1m 2","back_1m extra","back_2m","left_1m"};
    for (size_t i=0;i<sizeof invalid/sizeof invalid[0];++i) {
        reset();test_command("arm all",0);test_command(invalid[i],1);
        test_command("back_1m",2);test_tick(100);assert(count==8 && moves()==0);
    }
}

static void check_strafe_calibration_frames(const uint8_t directions[5])
{
    bool seen[5]={false};
    for (size_t i=0;i<count;++i) {
        if (packets[i][1]!=0xFD) continue;
        uint8_t id=packets[i][0];
        const uint8_t expected[]={id,0xFD,directions[id],0,5,10,0,0,0x3E,0x80,0,0,0x6B};
        assert(!seen[id]);seen[id]=true;
        assert(lengths[i]==sizeof expected && !memcmp(packets[i],expected,sizeof expected));
    }
    assert(moves()==4 && seen[1] && seen[2] && seen[3] && seen[4]);
}

static void test_strafe_calibration(void)
{
    /* 地址 1/2/3/4 的横移方向沿用已获实物确认的 X 型短动作。 */
    static const struct {
        const char *command,*short_command,*long_command;
        uint8_t directions[5];
    } cases[]={
        {"left_16000","left","left_long",{0,1,0,1,0}},
        {"right_16000","right","right_long",{0,0,1,0,1}}
    };
    for (size_t c=0;c<sizeof cases/sizeof cases[0];++c) {
        const char *cmd=cases[c].command;
        reset();test_command(cmd,0);test_tick(100);assert(count==8 && moves()==0);
        reset();test_command("arm 3",0);test_command(cmd,1);
        test_command("jog 1",2);test_tick(100);assert(count==8 && moves()==0);
        reset();test_command("arm all",0);test_command(cmd,10000);
        test_tick(10050);assert(count==8 && moves()==0);
        reset();test_command("arm all",0);test_command(cmd,1);
        test_tick(50);assert(count==12 && moves()==0);
        test_tick(51);check_strafe_calibration_frames(cases[c].directions);assert(count==16);
        char expected_log[200];
        snprintf(expected_log,sizeof expected_log,"CALIBRATION JOG sent: %s; 5 RPM, 16000 pulses, 80-second stop deadline.",cmd);
        assert(strstr(logs,expected_log));
        assert(!strstr(logs,"CALIBRATION JOG sent: forward_16000;"));
        test_command(cmd,52);test_command("arm all",53);assert(count==16);
        test_tick(3001);test_tick(8001);test_tick(80000);assert(count==16);
        test_tick(80001);assert(count==24);
        snprintf(expected_log,sizeof expected_log,"STOP/DISABLE sent after 80 seconds (%s).",cmd);
        assert(strstr(logs,expected_log));
        for (uint8_t id=1;id<=4;++id) {
            size_t at=16+2U*(id-1U);
            const uint8_t stop[]={id,0xFE,0x98,0,0x6B};
            const uint8_t disable[]={id,0xF3,0xAB,0,0,0x6B};
            assert(lengths[at]==sizeof stop && !memcmp(packets[at],stop,sizeof stop));
            assert(lengths[at+1]==sizeof disable && !memcmp(packets[at+1],disable,sizeof disable));
        }
        test_tick(160001);test_command(cmd,160002);assert(count==24);

        /* 长行程横移后，普通横移和前进测距不能继承它的方向/名称/时限。 */
        count=0;logs[0]=0;
        test_command("arm all",160100);test_command(cases[c].short_command,160101);test_tick(160151);
        check_group_pattern(cases[c].directions);
        test_tick(163100);assert(count==8);test_tick(163101);assert(count==16);
        assert(strstr(logs,"STOP/DISABLE sent after 3 seconds."));
        count=0;logs[0]=0;
        test_command("arm all",163200);test_command(cases[c].long_command,163201);test_tick(163251);
        check_long_pattern(cases[c].directions);
        test_tick(171200);assert(count==8);test_tick(171201);assert(count==16);
        assert(strstr(logs,"STOP/DISABLE sent after 8 seconds ("));
        count=0;logs[0]=0;
        test_command("arm all",171300);test_command("forward_16000",171301);test_tick(171351);
        check_calibration_forward_frames();assert(count==8);
        test_tick(251301);assert(count==16);
        assert(strstr(logs,"CALIBRATION JOG sent: forward_16000; 5 RPM, 16000 pulses, 80-second stop deadline."));
        assert(strstr(logs,"STOP/DISABLE sent after 80 seconds (forward_16000)."));

        reset();test_command("arm all",0);test_command(cmd,1);
        test_command("stop",51);test_tick(80001);assert(count==20 && moves()==0);
        reset();test_command("arm all",0);test_command(cmd,1);
        test_tick(51);test_command("stop",1000);test_tick(80001);assert(count==24 && moves()==4);
        reset();test_command("arm all",0);test_command(cmd,1);
        test_tick(80001);assert(count==20 && moves()==0);
        reset();test_command("arm all",0);test_command("probe 3",1);
        test_command(cmd,2);test_tick(100);assert(count==9 && moves()==0);
        reset();test_command("arm all",0);test_command(cmd,1);
        test_tick(51);test_fault();test_tick(80001);assert(count==24 && moves()==4);
        test_command(cmd,80002);assert(count==24);
        reset();test_command("arm all",0);fail_at=9;test_command(cmd,1);
        test_tick(100);assert(count==18 && moves()==0);
        reset();test_command("arm all",0);test_command(cmd,1);
        fail_at=13;test_tick(51);assert(count==22 && moves()==2);test_tick(80001);assert(count==22);
        reset();test_command("arm all",UINT32_MAX-100);
        test_command(cmd,UINT32_MAX-50);test_tick(0);check_strafe_calibration_frames(cases[c].directions);
        test_tick(79948);assert(count==16);test_tick(79949);assert(count==24);
        char invalid[40];snprintf(invalid,sizeof invalid,"%s extra",cmd);
        reset();test_command("arm all",0);test_command(invalid,1);
        test_command(cmd,2);test_tick(100);assert(count==8 && moves()==0);
    }
    const char *invalid[]={"left_160000","left_1m","right_1m","right_16000 100"};
    for (size_t i=0;i<sizeof invalid/sizeof invalid[0];++i) {
        reset();test_command("arm all",0);test_command(invalid[i],1);
        test_command("left_16000",2);test_tick(100);assert(count==8 && moves()==0);
    }
}

static void test_mecanum_commands(void)
{
    /* 按实物地址检查完整位置帧，预期值不从被测方向函数或配置宏计算：
     * 地址 1=右前、2=右后、3=左后、4=左前；1=该轮上沿朝车头。
     * 左横移：左前/右后反转；顺时针：右侧反转，左侧正转。 */
    static const struct { const char *command; uint8_t directions[5]; } cases[] = {
        {"forward", {0,1,1,1,1}}, {"back",  {0,0,0,0,0}},
        {"left",    {0,1,0,1,0}}, {"right", {0,0,1,0,1}},
        {"cw",      {0,0,0,1,1}}, {"ccw",   {0,1,1,0,0}}
    };
    for (size_t c = 0; c < sizeof cases / sizeof cases[0]; ++c) {
        const char *command = cases[c].command;
        reset(); test_command(command,0);test_tick(100);
        assert(count == 8 && moves() == 0); /* 未授权不发使能/动作。 */

        reset(); test_command("arm 3",0);test_command(command,1);
        test_command("jog 1",2);test_tick(100);
        assert(count == 8 && moves() == 0); /* 不允许单轮授权升级为四轮。 */

        reset(); test_command("arm all",0);test_command(command,1);
        test_tick(50);assert(count == 12 && moves() == 0);
        test_tick(51);check_group_pattern(cases[c].directions);
        test_command(command,52);assert(count == 16); /* 一次授权不能连发。 */
        test_tick(3001);assert(count == 24);
        for (uint8_t id=1; id<=4; ++id) {
            size_t at = 16 + 2U * (id-1U);
            const uint8_t stop[] = {id,0xFE,0x98,0,0x6B};
            const uint8_t disable[] = {id,0xF3,0xAB,0,0,0x6B};
            assert(lengths[at] == sizeof stop && !memcmp(packets[at],stop,sizeof stop));
            assert(lengths[at+1] == sizeof disable && !memcmp(packets[at+1],disable,sizeof disable));
        }
        test_command(command,3002);assert(count == 24);

        reset(); test_command("arm all",0);test_command(command,10000);test_tick(10050);
        assert(count == 8 && moves() == 0); /* 授权到期边界。 */
        reset(); test_command("arm all",0);test_command(command,1);
        test_command("stop",51);test_tick(1000);
        assert(count == 20 && moves() == 0); /* 停止优先，不能补发位置帧。 */
        reset(); test_command("arm all",0);test_command(command,1);test_tick(3001);
        assert(count == 20 && moves() == 0); /* 调度超过时限不补发。 */
        reset(); test_command("arm all",0);test_command("probe 3",1);
        test_command(command,2);test_tick(100);
        assert(count == 9 && moves() == 0); /* 查询撤销授权。 */
        reset(); test_command("arm all",0);test_command(command,1);test_fault();
        test_tick(1000);assert(count == 20 && moves() == 0);
    }
    reset(); test_command("arm all",0);test_command("left extra",1);
    test_command("left",2);test_tick(100);assert(count == 8);
    reset(); test_command("arm all",0);fail_at=9;test_command("cw",1);
    test_tick(100);assert(count == 18 && moves() == 0);
    reset(); test_command("arm all",0);test_command("left",1);
    fail_at=13;test_tick(51);assert(count == 22 && moves() == 2);
    test_tick(10000);assert(count == 22);
}

static void test_group_motion(void)
{
    reset(); test_command("forward",0);test_command("back",1);test_tick(100);
    assert(count == 8 && moves() == 0); /* 无授权不使能，也不运动。 */
    test_command("arm 3",101);test_command("forward",102);test_command("jog 0",103);
    assert(count == 8); /* 单轮授权不能升级成四轮；误用会撤销授权。 */
    test_command("arm all",104);test_command("jog 1",105);test_command("forward",106);
    assert(count == 8); /* 四轮授权不能与单轮命令混用。 */

    reset(); test_command("arm all",0); assert(count == 8);
    test_command("forward",1); assert(count == 12 && moves() == 0);
    for (size_t i=8;i<12;++i) {
        const uint8_t expected[] = {(uint8_t)(i-7),0xF3,0xAB,1,0,0x6B};
        assert(lengths[i] == sizeof expected && !memcmp(packets[i],expected,sizeof expected));
    }
    test_tick(50);assert(moves() == 0);
    test_tick(51);check_group_moves(1);assert(count == 16);
    test_command("back",52);test_command("arm all",53);assert(count == 16);
    test_tick(3001);assert(count == 24);
    test_command("forward",3002);assert(count == 24); /* 一次授权仅一次动作。 */

    reset(); test_command("arm all",0);test_command("back",1);test_tick(51);
    check_group_moves(0);

    reset(); test_command("arm all",0);test_command("forward",10000);test_tick(10050);
    assert(count == 8 && moves() == 0); /* 四轮授权超时也不使能。 */
    reset(); test_command("arm all",0);test_command("forward",1);
    test_command("stop",51);test_tick(1000);assert(count == 20 && moves() == 0);
    reset(); test_command("arm all",0);test_command("forward",1);
    test_tick(3001);assert(count == 20 && moves() == 0); /* 迟到不补发。 */

    reset(); test_command("arm all",0);fail_at=9;test_command("forward",1);
    test_tick(100);assert(count == 18 && moves() == 0); /* 第二台使能发送失败，全部停止。 */
    reset(); test_command("arm all",0);test_command("forward",1);
    fail_at=13;test_tick(51);assert(count == 22 && moves() == 2);
    test_tick(10000);assert(count == 22); /* 第二台位置发送失败，不再发余下两台。 */

    reset(); test_command("arm all",0);test_command("forward",1);test_fault();
    size_t after_fault=count;test_tick(1000);test_fault();assert(count == after_fault && moves() == 0);
    reset(); test_command("arm all",0);test_command("probe 3",1);
    test_command("forward",2);test_tick(100);assert(count == 9 && moves() == 0);
    reset(); test_command("arm all",0);test_command("forwad",1);
    test_command("forward",2);assert(count == 8);

    reset(); test_command("arm all",UINT32_MAX-100);test_command("forward",UINT32_MAX-50);
    test_tick(0);check_group_moves(1);test_tick(2949);assert(count == 24);
}

int main(void)
{
    reset();
    assert(count == 8 && moves() == 0);
    test_tick(100000); test_command("jog 0", 100001); test_tick(100060);
    assert(moves() == 0); /* 上电/无确认都不能动作。 */

    reset();
    test_command("arm 1", 0); assert(count == 8);
    test_command("jog 0", 1); assert(count == 9);
    test_tick(50); assert(moves() == 0);
    test_tick(51); assert(moves() == 1);
    const uint8_t expected[] = {1,0xFD,0,0,5,10,0,0,0,160,0,0,0x6B};
    assert(lengths[9] == sizeof expected && !memcmp(packets[9], expected, sizeof expected));
    test_command("arm 2", 60); test_command("jog 1", 61);
    assert(moves() == 1); /* 运行时不能切轮重启。 */
    test_tick(3001); assert(count == 18);
    test_command("jog 0", 3002); assert(moves() == 1); /* 一次授权只能用一次。 */

    reset();
    test_command("arm 1", 0); test_command("jog 0", 10000); test_tick(10050);
    assert(moves() == 0 && count == 8); /* 授权到期边界。 */
    test_command("arm 0", 11000); test_command("jog 0", 11001);
    test_command("arm 5", 12000); test_command("jog 0", 12001);
    assert(count == 8);
    test_command("arm 1", 13000); test_command("jog 0 extra", 13001);
    test_command("jog 0", 13002); assert(count == 8);

    reset();
    test_command("arm 4", 0); test_command("jog 1", 1);
    test_command("stop", 51); test_tick(500); assert(moves() == 0);

    reset();
    test_command("arm 4", 0); test_command("jog 1", 1); test_tick(3001);
    assert(moves() == 0); /* 调度延迟超过期限时，不补发迟到的运动命令。 */

    reset();
    test_command("arm 3", 0); fail_next = true; test_command("jog 0", 1);
    test_tick(60); assert(moves() == 0); /* 使能发送失败时不继续。 */
    test_command("arm 3", 70); test_command("jog 0", 71);
    fail_next = true; test_tick(121); /* 位置发送失败会发停止并清除授权。 */
    size_t before = count; test_command("jog 0", 122); assert(count == before);

    reset();
    test_command("arm 2", UINT32_MAX-100); test_command("jog 1", UINT32_MAX-50);
    test_tick(0); assert(moves() == 1 && packets[9][0] == 2 && packets[9][2] == 1);
    test_tick(2949); assert(count == 18); /* 毫秒计数回卷。 */
    test_command("arm 1", 4000); test_fault();
    test_command("jog 0", 4001); assert(moves() == 1);
    reset();
    test_command("arm 3", 0);
    test_command("probe 3", 1);
    const uint8_t query[] = {3,0x1F,0x6B};
    assert(count == 9 && lengths[8] == sizeof query && !memcmp(packets[8], query, sizeof query));
    test_command("jog 0", 2); assert(count == 9); /* 查询撤销动作授权。 */
    const uint8_t reply[] = {3,0x1F,0xF4,0x78,0x6B};
    receive(reply, 2, 20); assert(!strstr(logs,"PROBE OK"));
    receive(reply+2, 3, 30); assert(strstr(logs,"PROBE OK"));
    test_tick(501); assert(!strstr(logs,"PROBE timeout") && moves() == 0);

    reset(); test_command("probe 3", 0);
    const uint8_t other[] = {2,0x1F,0xF4,0x78,0x6B};
    receive(other, sizeof other, 20);
    receive(query, sizeof query, 30); /* 只有本地回显不算完整回复。 */
    test_tick(500); assert(strstr(logs,"PROBE timeout") && !strstr(logs,"PROBE OK"));
    assert(count == 9 && moves() == 0);

    reset(); test_command("probe 3", UINT32_MAX-100);
    receive(reply,sizeof reply,10);test_tick(600);
    assert(strstr(logs,"PROBE OK") && !strstr(logs,"PROBE timeout"));

    reset(); test_command("probe 3", 0);
    const uint8_t error[] = {3,0x00,0xEE,0x6B};
    receive(error,sizeof error,30);test_tick(600);
    assert(strstr(logs,"PROBE rejected") && !strstr(logs,"PROBE timeout"));

    reset(); test_command("probe 3", 0);
    receive(reply,sizeof reply,500);
    assert(strstr(logs,"PROBE timeout") && !strstr(logs,"PROBE OK"));

    reset(); fail_next = true; test_command("probe 3", 0);test_tick(600);
    assert(strstr(logs,"PROBE TX failed") && !strstr(logs,"PROBE timeout") && count == 9);
    reset(); test_command("probe 0",0);test_command("probe 5",0);
    assert(count == 8);
    test_command("arm 3",1);test_command("jog 0",2);
    test_command("probe 3",3); assert(count == 9 && strstr(logs,"BUSY"));
    test_command("stop",4);test_tick(1000);assert(moves() == 0);
    reset(); test_command("arm 3",0); test_command("jog 0",1);
    test_fault(); size_t after_fault = count;
    for (unsigned i=0;i<100;++i) test_fault();
    test_tick(100); assert(count == after_fault && moves() == 0);
    test_command("jog 0",101); assert(count == after_fault);
    test_command("arm 3",102); test_command("jog 0",103);
    test_fault(); assert(count == after_fault + 9); /* 重新授权后，错误仍立即停机。 */
    test_tick(200); assert(moves() == 0);
    test_command("probe 3",201); test_fault(); after_fault = count;
    test_tick(1000); test_fault(); assert(count == after_fault && moves() == 0);
    test_group_motion();
    test_mecanum_commands();
    test_long_left();
    test_long_right();
    test_six_long_commands();
    test_calibration_forward();
    test_forward_1m();
    test_back_1m();
    test_strafe_calibration();
    reset(); test_command("arm all", 0);
    assert(test_begin_diagnostic());
    test_command("left_16000", 1); test_tick(100);
    assert(count == 8 && moves() == 0); /* IMU诊断撤销尚未用的整车授权。 */
    reset(); test_command("arm 3", 0);
    assert(test_begin_diagnostic());
    test_command("jog 1", 1); test_tick(100); assert(count == 8 && moves() == 0);
    reset(); test_command("arm all", 0); test_command("left_16000", 1);
    assert(!test_begin_diagnostic() && strstr(logs, "BUSY: stop before IMU"));
    test_command("stop", 2); test_tick(100); assert(moves() == 0);
    assert(test_begin_diagnostic());
    reset(); test_command("probe 3", 0); assert(test_begin_diagnostic());
    test_tick(600); assert(!strstr(logs, "PROBE timeout") && moves() == 0);
    reset(); test_command("arm all", 0);
    for (unsigned i=0; i<100; ++i) assert(!test_is_moving());
    assert(count == 8); /* 观察状态不会撤销授权或发送电机帧。 */
    test_command("left_16000", 1); assert(!test_is_moving());
    test_tick(51); assert(test_is_moving() && moves() == 4);
    size_t after_moves = count;
    for (unsigned i=0; i<100; ++i) assert(test_is_moving());
    assert(count == after_moves);
    test_tick(80001); assert(!test_is_moving());
    reset();test_command("arm 3", 0);test_command("jog 1", 1);test_tick(51);
    assert(test_is_moving());test_command("stop", 52);assert(!test_is_moving());
    puts("PASS: short/long patterns, straight/strafe calibration and 1m trials; exact frames, authorization, failures, stop/timeouts and profile isolation.");
}
