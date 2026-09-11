// 挂钩舵机钩子 — MG996R 直驱
// OpenSCAD 打开此文件 → F6 渲染 → Export STL
// 单位: mm

/* ===== 可调参数 ===== */
HOOK_LENGTH    = 50;    // 钩身总长 (舵机轴心→钩尖)
HOOK_WIDTH     = 15;    // 钩身宽度
HOOK_THICK     = 5;     // 钩身厚度 (平放打印)
TIP_DROP       = 10;    // 钩头下探 (Z轴高低差)
TIP_WIDTH      = 2.5;   // 钩尖宽度 (适配3~4mm从车槽口)
TIP_LENGTH     = 8;     // 钩尖长度 (伸入槽口部分)
CURVE_R        = 5;     // 过渡圆弧半径
SHAFT_DIAM     = 5.8;   // 中心孔径 (略小于舵机轴6mm, 压入咬紧)
SCREW_DIAM     = 3.2;   // M3沉头螺丝孔
SCREW_HEAD_DIAM = 6.5;  // M3沉头直径
SCREW_HEAD_DEPTH = 2.5; // M3沉头深度
$fn            = 64;    // 圆弧精度

/* ===== 主模型 ===== */
module hook() {
    difference() {
        union() {
            // —— 钩身平板 ——
            // 舵机轴心在原点，钩子沿 +X 方向伸出
            translate([HOOK_LENGTH/2, 0, 0])
                cube([HOOK_LENGTH, HOOK_WIDTH, HOOK_THICK], center=true);

            // —— 弧形过渡 (宽度渐变: 15→2.5) ——
            // 用 hull() 连接钩身末端和钩尖起点的椭圆截面
            hull() {
                // 钩身末端 (宽15, 厚5)
                translate([HOOK_LENGTH - CURVE_R, 0, HOOK_THICK/2])
                    cube([0.1, HOOK_WIDTH, HOOK_THICK], center=true);
                // 钩尖起点 (宽2.5, 厚5, 下探开始)
                translate([HOOK_LENGTH, 0, HOOK_THICK/2])
                    cube([0.1, TIP_WIDTH, HOOK_THICK], center=true);
            }

            // —— 钩尖下探 (垂直柱) ——
            translate([HOOK_LENGTH, 0, -TIP_DROP/2 + HOOK_THICK/2])
                cube([TIP_WIDTH, TIP_WIDTH, TIP_DROP - HOOK_THICK/2], center=true);

            // —— 钩尖水平段 (伸入槽口的部分) ——
            translate([HOOK_LENGTH + TIP_LENGTH/2, 0, -TIP_DROP + HOOK_THICK/2 + TIP_WIDTH/2])
                cube([TIP_LENGTH, TIP_WIDTH, TIP_WIDTH], center=true);

            // —— 钩尖回钩 (防滑脱的小倒钩) ——
            translate([HOOK_LENGTH + TIP_LENGTH - 1, 0, -TIP_DROP + HOOK_THICK/2 - 0.5])
                rotate([0, 30, 0])
                    cube([3, TIP_WIDTH, TIP_WIDTH], center=true);
        }

        // —— 中心孔 (舵机轴 φ5.8mm) ——
        translate([0, 0, -1])
            cylinder(d=SHAFT_DIAM, h=HOOK_THICK + 2);

        // —— M3 沉头螺丝孔 (从上表面贯穿) ——
        translate([0, 0, HOOK_THICK - SCREW_HEAD_DEPTH])
            cylinder(d=SCREW_HEAD_DIAM, h=SCREW_HEAD_DEPTH + 0.1);
        translate([0, 0, -1])
            cylinder(d=SCREW_DIAM, h=HOOK_THICK + 2);

        // —— 减重槽 (可选, 勾身中间挖空) ——
        translate([HOOK_LENGTH * 0.45, 0, HOOK_THICK/2])
            cube([HOOK_LENGTH * 0.5, HOOK_WIDTH - 6, HOOK_THICK + 1], center=true);
    }
}

// 渲染
hook();

/* ===== 打印说明 =====
 * 摆放: 平放在热床上 (钩身水平, 钩尖朝上悬空)
 *       Z轴层纹垂直于钩身 → 强度最优
 * 支撑: 钩尖下探10mm悬空 → 必须开支撑 (树状支撑, 仅接触热床)
 * 材料: PLA 或 PETG (PETG韧性更好)
 * 填充: 100% (实心)
 * 壁厚: 至少3层 (0.4mm喷嘴 = 1.2mm壁厚)
 */
