// Step 5: random vibration, GEVS qualification level (GSFC-STD-7000B, 14.1 grms), one axis per run.
// Support: rail ends at Z = 0 and Z = 340.5 fixed (CubeSat clamped by the test-pod / deployer end faces);
// the base excitation drives all fixtures together. Modal damping 5 % (Q = 10, as in the step-1 Miles
// estimate). Output: 1-sigma RMS von Mises / displacement / acceleration, the location and component of the
// peaks, and 3-sigma values for the margin check (3-sigma vs yield with FS 1.25, vs ultimate with FS 1.4).
// Env: RV_AXIS = X | Y | Z, RV_FMAX = 2000 (upper bound for modes), RV_H = 4, RV_ASM.
using System;
using System.IO;
using System.Linq;
using System.Globalization;
using System.Collections.Generic;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.cosworks;

public static class StepRV
{
    static readonly string Asm = @"D:\cad free projects\CubeSat 3U\" + (System.Environment.GetEnvironmentVariable("RV_ASM") ?? "CubeSat_3U_v4.SLDASM");
    static TextWriter log;
    static void P(string s) { Console.WriteLine(s); if (log != null) { log.WriteLine(s); log.Flush(); } }
    static double[] ToD(object o) { return ((Array)o).Cast<object>().Select(x => Convert.ToDouble(x)).ToArray(); }

    // GEVS qualification ASD (g^2/Hz): 20 Hz 0.026, +6 dB/oct to 50 Hz 0.16, flat to 800 Hz, -6 dB/oct to 2000 Hz 0.026
    static double Gevs(double f)
    {
        if (f <= 50) return 0.026 * Math.Pow(f / 20.0, Math.Log(0.16 / 0.026) / Math.Log(50.0 / 20.0));
        if (f <= 800) return 0.16;
        return 0.16 * Math.Pow(f / 800.0, Math.Log(0.026 / 0.16) / Math.Log(2000.0 / 800.0));
    }

    public static void Run(string outDir)
    {
        var ci = CultureInfo.InvariantCulture;
        char ax = (System.Environment.GetEnvironmentVariable("RV_AXIS") ?? "Y")[0];
        double fmax = double.Parse(System.Environment.GetEnvironmentVariable("RV_FMAX") ?? "2000", ci);
        double h = double.Parse(System.Environment.GetEnvironmentVariable("RV_H") ?? "4", ci);
        string tag = System.Environment.GetEnvironmentVariable("RV_TAG") ?? "v4";
        log = new StreamWriter(Path.Combine(outDir, "step5_rv_" + tag + "_" + ax + ".txt"));
        P("Step 5 random vibration | GEVS qual | axis " + ax + " | " + Path.GetFileName(Asm) + " | modes to " + fmax + " Hz | h = " + h + " | " + DateTime.Now);

        // input curve: log-spaced points (SOLIDWORKS interpolation between points is then irrelevant)
        var fs = new List<double>();
        for (int i = 0; i <= 60; i++) fs.Add(20.0 * Math.Pow(100.0, i / 60.0));
        fs.AddRange(new[] { 50.0, 800.0 }); fs = fs.Distinct().OrderBy(v => v).ToList();
        double grms2 = 0;
        for (int i = 1; i < fs.Count; i++) grms2 += 0.5 * (Gevs(fs[i]) + Gevs(fs[i - 1])) * (fs[i] - fs[i - 1]);
        P(string.Format("input: {0} points, {1:F2} grms (trapezoid on the log-spaced points; GEVS 14.1)", fs.Count, Math.Sqrt(grms2)));

        var sw = SwSim.Connect(log);
        int e = 0, w = 0;
        var doc = (ModelDoc2)sw.GetOpenDocumentByName(Asm) ?? (ModelDoc2)sw.OpenDoc6(Asm, 2, 0, "", ref e, ref w);
        sw.ActivateDoc3(doc.GetTitle(), false, 0, ref e);
        var cw = SwSim.Simulation(sw, log);
        var sm = cw.ActiveDoc.StudyManager;
        var comps = ((object[])((AssemblyDoc)doc).GetComponents(true)).Cast<Component2>().ToList();
        P(string.Format("components {0}, mass {1:F3} kg", comps.Count, ((MassProperty)doc.Extension.CreateMassProperty()).Mass));

        var ends = new List<object>();
        foreach (var c in comps.Where(c => c.Name2.StartsWith("side_frame")))
            foreach (Body2 b in (object[])c.GetBodies2(0))
                foreach (Face2 f in (object[])b.GetFaces())
                {
                    if (!((Surface)f.GetSurface()).IsPlane()) continue;
                    double[] bx = (double[])f.GetBox();
                    foreach (double zf in new[] { 0.0, 0.3405 })
                        if (Math.Abs(bx[2] - zf) < 1e-7 && Math.Abs(bx[5] - zf) < 1e-7) ends.Add(f);
                }
        P("fixed rail-end faces: " + ends.Count);

        string name = "RV_GEVSq_" + ax + "_" + tag;
        var existing = new HashSet<string>();
        for (int i = 0; i < sm.StudyCount; i++) { var s0 = sm.GetStudy(i); if (s0 != null) existing.Add(s0.Name); }
        for (int k = 2; existing.Contains(name); k++) name = "RV_GEVSq_" + ax + "_" + tag + "_r" + k;
        int err;
        CWStudy st = sm.CreateNewStudy3(name, 8, 2, out err);   // dynamic, random vibration
        if (st == null) throw new Exception("study creation failed " + err);
        P("study " + name + " subtype " + st.DynamicAnalysisSubType);
        var o = st.DynamicStudyOptions;
        P("  frequency option err " + o.SetFrequencyOption2(1, fmax));
        P("  RV f range err " + o.SetRandomVibrationFrequencyUnits2(1) + "/" + o.SetRandomVibrationFrequencyLowerLimit2(20) + "/" + o.SetRandomVibrationFrequencyUpperLimit2(2000));
        P("  RV points err " + o.SetRandomVibrationNoOfFrequencyPoints2(40) + ", correlation err " + o.SetRandomVibrationCorrelationOption2(0));
        var dmp = st.DampingOptions;
        dmp.DampingType = 0;
        P("  damping err " + dmp.SetDampingRatios(1, new double[] { 1, 10000, 0.05 }));
        st.ContactManager.SetGlobalContact(0, 0);
        var lr = st.LoadsAndRestraintsManager;
        lr.AddRestraint(0, ends.ToArray(), null, out err); P("  fixture err " + err);
        var front = (Feature)((AssemblyDoc)doc).FeatureByName("Front Plane");
        CWBaseExcitation bex = lr.AddUniformBaseExcitation2(2, front, 3, ax == 'X', ax == 'X' ? 1 : 0, ax == 'Y', ax == 'Y' ? 1 : 0, ax == 'Z', ax == 'Z' ? 1 : 0, out err);
        P("  base excitation err " + err);
        if (bex == null) throw new Exception("base excitation failed");
        bex.BaseExcitationBeginEdit();
        var flat = new List<double>(); foreach (var f in fs) { flat.Add(f); flat.Add(Gevs(f)); }
        int ce; bool ok = bex.SetTimeOrFrequencyCurve2(flat.ToArray(), out ce);
        if (!ok || ce != 0)
        {
            var withN = new List<double> { fs.Count }; withN.AddRange(flat);
            ok = bex.SetTimeOrFrequencyCurve2(withN.ToArray(), out ce);
            P("  curve (count-prefixed) ok " + ok + " err " + ce);
        }
        else P("  curve ok, err " + ce);
        P("  base excitation end edit " + bex.BaseExcitationEndEdit());
        object cv = bex.GetTimeOrFrequencyCurve();
        if (cv != null && !(cv is DBNull)) { var c2 = ToD(cv); P("  curve read back: " + c2.Length + " values, first " + string.Join(", ", c2.Take(6).Select(v => v.ToString("G4", ci)).ToArray())); }

        var dummies = comps.Where(c => c.Name2.StartsWith("camera") || c.Name2.StartsWith("rw_wheel")
                                    || c.Name2.StartsWith("battery_cell") || c.Name2.StartsWith("antenna")).Cast<object>().ToArray();
        var thin = comps.Where(c => c.Name2.StartsWith("shear_panel")).Cast<object>().ToArray();
        var mesh = st.Mesh; mesh.MesherType = 0; mesh.Quality = 1;
        var mc = mesh.ApplyMeshControl(dummies, out err); if (mc != null) { mc.Units = 0; mc.ElementSize = 3 * h; }
        var mc2 = mesh.ApplyMeshControl(thin, out err); if (mc2 != null) { mc2.Units = 0; mc2.ElementSize = Math.Min(h, 2.0); }
        var t0 = DateTime.Now;
        int me = st.CreateMesh(0, h, h * 0.05);
        P(string.Format("  mesh err {0}, nodes {1}, {2:F0} s", me, mesh.NodeCount, (DateTime.Now - t0).TotalSeconds));
        if (me != 0 || mesh.GetNoOfFailedComponents() > 0) throw new Exception("meshing failed");
        doc.Save3(1, ref e, ref w);
        t0 = DateTime.Now;
        int re = st.RunAnalysis();
        P(string.Format("  solve err {0}, {1:F0} s", re, (DateTime.Now - t0).TotalSeconds));
        if (re != 0) { doc.Save3(1, ref e, ref w); return; }
        var res = st.Results;

        // 1-sigma RMS results
        double[] s = ToD(res.GetMinMaxStressRMS(9, 0, null, 3, out err));   // von Mises, N/mm^2
        P("  RMS von Mises raw (err " + err + "): " + string.Join(", ", s.Select(v => v.ToString("G5", ci)).ToArray()));
        double[] d = ToD(res.GetMinMaxDisplacementRMS(3, null, 0, out err));  // URES, mm
        P("  RMS displacement raw (err " + err + "): " + string.Join(", ", d.Select(v => v.ToString("G5", ci)).ToArray()));
        Report(st, comps, "stress", (int)s[2], s[3], "MPa");
        Report(st, comps, "displacement", (int)d[2], d[3], "mm");
        try
        {
            double[] a = ToD(res.GetMinMaxAcceleration(3, 1, null, 3, out err));
            P("  acceleration raw (err " + err + "): " + string.Join(", ", a.Select(v => v.ToString("G5", ci)).ToArray()));
        }
        catch (Exception ex) { P("  acceleration not available: " + ex.Message); }

        // per-component peak RMS von Mises (by node, over each component's faces)
        var groups = new Dictionary<string, double>();
        foreach (var c in comps)
        {
            var faces = new List<object>();
            foreach (Body2 b in (object[])c.GetBodies2(0)) faces.AddRange((object[])b.GetFaces());
            object sv = null;
            try { sv = res.GetStressForEntities2(true, 9, 1, null, faces.ToArray(), 3, out err); } catch { }
            if (sv == null || sv is DBNull) continue;
            double[] vv = ToD(sv); double mx = 0;
            for (int i = 1; i < vv.Length; i += 2) mx = Math.Max(mx, vv[i]);
            string key = c.Name2.Substring(0, c.Name2.LastIndexOf('-'));
            if (!groups.ContainsKey(key) || groups[key] < mx) groups[key] = mx;
        }
        if (groups.Count > 0)
        {
            P("  per component (value from GetStressForEntities2 step 1 - check it equals the RMS global max for the top part):");
            foreach (var kv in groups.OrderByDescending(kv => kv.Value).Take(15))
                P(string.Format("    {0,-18} {1,8:F2} MPa (3-sigma {2,8:F1})", kv.Key, kv.Value, 3 * kv.Value));
        }
        doc.Save3(1, ref e, ref w);
        P("done " + DateTime.Now);
        log.Close();
    }

    static void Report(CWStudy st, List<Component2> comps, string what, int node, double v, string unit)
    {
        double x, y, z; st.Mesh.GetNodeLocation(node, out x, out y, out z);
        string hit = "";
        foreach (var c in comps)
        {
            var b = c.GetBox(false, false) as double[];
            if (b != null && x >= b[0] - 1e-4 && x <= b[3] + 1e-4 && y >= b[1] - 1e-4 && y <= b[4] + 1e-4 && z >= b[2] - 1e-4 && z <= b[5] + 1e-4)
                hit += (hit == "" ? "" : "|") + c.Name2;
        }
        P(string.Format("  max RMS {0}: {1:G4} {2} (3-sigma {3:G4}) at ({4:F1}, {5:F1}, {6:F1}) mm in {7}", what, v, unit, 3 * v, x * 1e3, y * 1e3, z * 1e3, hit));
    }
}
