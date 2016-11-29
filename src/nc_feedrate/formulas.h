#ifndef FORMULAS_H
#define FORMULAS_H

namespace formulas {
// http://www.sandvik.coromant.com/en-us/knowledge/milling/formulas_and_definitions/formulas

/* Dcap/mm      - Cutter diameter at actual depth of cut
 * fz/mm        - feed per tooth
 * Zn           - total cutter teeth
 * Zc           - effective cutter teeth
 * Vf/mm/min    - table feed
 * fn/mm        - feed per revolution
 * ap/mm        - depth of cut
 * Vc/m/min     - Cutting speed
 * Y0           - chip rake angle
 * ae/mm        - working engagement
 * n/rpm        - spindle speed
 * Pc/kW        - net power
 * Mc/Nm        - Torque
 * Q/cm3/min    - Material removal rate
 * hm/mm        - Average chip thickness
 * hex/mm       - Max chip thickness
 * Kr/deg       - Entering angle
 * Dm/mm        - Machined diameter (component diameter)
 * Dw/mm        - Unmachined diameter
 * Vfm/mm/min   - Table feed of tool at Dm (machined diameter)
 */

double Vc(double Dcap, double n) {
    return (Dcap * PI * n) / 1000.0;
}

double n(double Vc, double Dcap) {
    return (Vc * 1000.0) / (PI * Dcap);
}

double fz(double Vf, double n, unsigned Zc) {
    return Vf / (n * Zc);
}

double Q(double ap, double ae, double Vf) {
    return (ap * ae * Vf) / 1000.0;
}

double Vf(double fz, double n, double Zc) {
    return fz * n * Zc;
}

double Mc(double Pc, double n) {
    return (Pc * 30.0 * 1000.0) / (PI * n);
}

double Pc(double ap, double ae, double Vf, double kc) {
    return (ap * ae * Vf * kc) / (60 * 1000000.0);
}

double hm_side(double Kr, double ae, double fz, double Dcap) {
    auto deg2rad = [](double d) { return (d / 180.0) * PI; };
    return (360 * std::sin(deg2rad(Kr)) * ae * fz) / (PI * Dcap * std::acos(deg2rad(1- ((2 * ae) / Dcap) )));
}
double hm_face(double Kr, double ae, double fz, double Dcap) {
    auto deg2rad = [](double d) { return (d / 180.0) * PI; };
    return (180 * std::sin(deg2rad(Kr)) * ae * fz) / (PI * Dcap * std::asin(deg2rad(ae/Dcap)));
}

}

#endif
