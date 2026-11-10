// Random-vibration benchmark against a hand solution: the step-0 cantilever (20 x 20 x 200 mm 6061-T6, fixed at
// z = 0, f1 = 408 Hz) under the same GEVS input, damping and API settings as StepRV, excited in X.
// Hand (Miles + cantilever mode-1 participation): tip RMS acceleration = 1.566 * sqrt(pi/2 * f1 * Q * ASD(f1))
// with Q = 1/(2 zeta); root RMS bending stress from the mode-1 tip-displacement relation
//   sigma_root = 3.516^2/... -> computed in the console with the exact modal formula below.
// Env: RB_ZETA (default 0.05), RB_METHOD (0 standard / 1 approximate).
using System;
using System.IO;
using System.Linq;
using System.Globalization;
using System.Collections.Generic;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.cosworks;

public static class RVBench
{
    static double Gevs(double f)
    {
        if (f <= 50) return 0.026 * Math.Pow(f / 20.0, Math.Log(0.16 / 0.026) / Math.Log(50.0 / 20.0));
        if (f <= 800) return 0.16;
        return 0.16 * Math.Pow(f / 800.0, Math.Log(0.026 / 0.16) / Math.Log(2000.0 / 800.0));
    }
    static double[] ToD(object o) { return ((Array)o).Cast<object>().Select(x => Convert.ToDouble(x)).ToArray(); }

    public static void Run(string outDir)
    {
        var ci = CultureInfo.InvariantCulture;
        double zeta = double.Parse(System.Environment.GetEnvironmentVariable("RB_ZETA") ?? "0.05", ci);
        int meth = int.Parse(System.Environment.GetEnvironmentVariable("RB_METHOD") ?? "0");
        var sw = SwSim.Connect(Console.Out);
        int e = 0, w = 0;
        string part = Path.Combine(outDir, "step0_cantilever.SLDPRT");
        var doc = (ModelDoc2)sw.OpenDoc6(part, 1, 0, "", ref e, ref w);
        sw.ActivateDoc3(doc.GetTitle(), false, 0, ref e);
        var cw = SwSim.Simulation(sw, Console.Out);
        var sm = cw.ActiveDoc.StudyManager;
        string name = "RVBench_z" + zeta.ToString(ci).Replace(".", "p") + "_m" + meth + "_" + DateTime.Now.ToString("HHmmss");
        int err;
        CWStudy st = sm.CreateNewStudy3(name, 8, 2, out err);
        var o = st.DynamicStudyOptions;
        o.SetFrequencyOption2(0, 4);
        o.SetRandomVibrationFrequencyLowerLimit2(20); o.SetRandomVibrationFrequencyUpperLimit2(2000);
        o.SetRandomVibrationNoOfFrequencyPoints2(40); o.SetRandomVibrationCorrelationOption2(0); o.SetRandomVibrationAnalysisMethod2(meth);
        var dmp = st.DampingOptions; dmp.DampingType = 0; dmp.ComputeFromMaterialDamping2 = false;
        Console.WriteLine("damping err " + dmp.SetDampingRatios(1, new object[] { 1, 1000, zeta }));
        object[] root = SwSim.FacesOnPlane(doc, 2, 0.0, 1e-9);
        var lr = st.LoadsAndRestraintsManager;
        lr.AddRestraint(0, root, null, out err); Console.WriteLine("fixture err " + err + " faces " + root.Length);
        var front = (Feature)((PartDoc)doc).FeatureByName("Front Plane");
        var bex = lr.AddUniformBaseExcitation2(2, front, 3, true, 1, false, 0, false, 0, out err);
        double[] gf = { 20, 50, 800, 2000, 3000 };
        var cd = new List<object> { gf.Length };
        foreach (double f in gf) { cd.Add(2 * Math.PI * f); cd.Add(Gevs(f)); }
        bex.BaseExcitationBeginEdit(); int ce; bex.SetTimeOrFrequencyCurve2(cd.ToArray(), out ce); bex.BaseExcitationEndEdit();
        st.Mesh.MesherType = 0; st.Mesh.Quality = 1;
        Console.WriteLine("mesh err " + st.CreateMesh(0, 3.0, 0.15) + " nodes " + st.Mesh.NodeCount);
        Console.WriteLine("solve err " + st.RunAnalysis());
        var r = st.Results;
        double[] s = ToD(r.GetMinMaxStressRMS(9, 0, null, 3, out err));
        double[] sx = ToD(r.GetMinMaxStressRMS(2, 0, null, 3, out err));   // SZ = bending stress along the beam
        double x, y, z; st.Mesh.GetNodeLocation((int)s[2], out x, out y, out z);
        Console.WriteLine(string.Format("FEA: RMS von Mises max {0:F2} MPa at z = {1:F1} mm; RMS SZ max {2:F2} MPa", s[3], z * 1e3, sx[3]));

        // hand solution (Euler-Bernoulli cantilever, mode 1 only)
        double E = 69e9, rho = 2700, a = 0.02, L = 0.2, I = Math.Pow(a, 4) / 12, A = a * a, c = a / 2;
        double b1 = 1.8751041, f1 = b1 * b1 / (2 * Math.PI * L * L) * Math.Sqrt(E * I / (rho * A));
        double Q = 1 / (2 * zeta), g = 9.80665;
        double sdofRms = Math.Sqrt(Math.PI / 2 * f1 * Q * Gevs(f1)) * g;            // m/s^2, SDOF absolute accel
        // mode shape phi(x) = cosh - cos - s(sinh - sin), s = (cosh b + cos b)/(sinh b + sin b); phi(L) = 2
        double sg = (Math.Cosh(b1) + Math.Cos(b1)) / (Math.Sinh(b1) + Math.Sin(b1));
        // participation Gamma = int(phi)/int(phi^2) with int(phi^2) = L, int(phi) = 0.7830*L (normalised phi(L)=2)
        double Gam = 0.7830;
        double w1 = 2 * Math.PI * f1;
        double qrms = Gam * sdofRms / (w1 * w1);   // modal coordinate RMS (relative), phi normalised to tip = 2
        double tipRel = 2 * qrms;
        double curv0 = (b1 / L) * (b1 / L) * 2;     // phi''(0) = (b/L)^2 * (1 + 1) for this normalisation
        double sigRoot = E * c * curv0 * qrms / 1e6;
        Console.WriteLine(string.Format("hand: f1 {0:F1} Hz, Q {1:F1}, SDOF {2:F1} g rms, tip rel. disp {3:F4} mm rms, root bending stress {4:F2} MPa rms",
            f1, Q, sdofRms / g, tipRel * 1e3, sigRoot));
        Console.WriteLine(string.Format("FEA / hand (stress) = {0:F3}", sx[3] / sigRoot));
    }
}
