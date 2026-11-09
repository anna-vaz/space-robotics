// Linear buckling of CubeSat_3U_v3 under 10 g in one axis (cantilever BC: rail ends at Z = 0 fixed).
// Output: buckling load factors (BLF = multiple of the 10 g load at which the structure buckles) and,
// per mode, the component where the buckled shape is largest.
// Required (step 1): BLF >= 3-sigma random load x 1.4 / 10 g: lateral 76 g -> 10.6, axial 112 g -> 15.7.
// Env: SB_AXIS = X | Y | Z (one per SOLIDWORKS session: later solves pop up a warning dialog), SB_N = 4, SB_H = 4.
using System;
using System.IO;
using System.Linq;
using System.Globalization;
using System.Collections.Generic;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.cosworks;

public static class StepBuckle
{
    const string Asm = @"D:\cad free projects\CubeSat 3U\CubeSat_3U_v3.SLDASM";
    static TextWriter log;
    static void P(string s) { Console.WriteLine(s); if (log != null) { log.WriteLine(s); log.Flush(); } }
    static double[] ToD(object o) { return ((Array)o).Cast<object>().Select(x => Convert.ToDouble(x)).ToArray(); }

    public static void Run(string outDir)
    {
        var ci = CultureInfo.InvariantCulture;
        char ax = (System.Environment.GetEnvironmentVariable("SB_AXIS") ?? "Y")[0];
        int nModes = int.Parse(System.Environment.GetEnvironmentVariable("SB_N") ?? "4");
        double h = double.Parse(System.Environment.GetEnvironmentVariable("SB_H") ?? "4", ci);
        log = new StreamWriter(Path.Combine(outDir, "stepB_buckling_v3_" + ax + ".txt"));
        P("Buckling | 10 g " + ax + " | cantilever | h = " + h + " mm | " + nModes + " modes | " + DateTime.Now);
        var sw = SwSim.Connect(log);
        int e = 0, w = 0;
        var doc = (ModelDoc2)sw.GetOpenDocumentByName(Asm) ?? (ModelDoc2)sw.OpenDoc6(Asm, 2, 0, "", ref e, ref w);
        sw.ActivateDoc3(doc.GetTitle(), false, 0, ref e);
        var cw = SwSim.Simulation(sw, log);
        var sm = cw.ActiveDoc.StudyManager;
        var comps = ((object[])((AssemblyDoc)doc).GetComponents(true)).Cast<Component2>().ToList();

        var endZ0 = new List<object>();
        foreach (var c in comps.Where(c => c.Name2.StartsWith("side_frame")))
            foreach (Body2 b in (object[])c.GetBodies2(0))
                foreach (Face2 f in (object[])b.GetFaces())
                {
                    if (!((Surface)f.GetSurface()).IsPlane()) continue;
                    double[] bx = (double[])f.GetBox();
                    if (Math.Abs(bx[2]) < 1e-7 && Math.Abs(bx[5]) < 1e-7) endZ0.Add(f);
                }
        P("fixed rail-end faces: " + endZ0.Count);

        string name = "Buckle10g_" + ax + "_v3";
        var existing = new HashSet<string>();
        for (int i = 0; i < sm.StudyCount; i++) { var s0 = sm.GetStudy(i); if (s0 != null) existing.Add(s0.Name); }
        for (int k = 2; existing.Contains(name); k++) name = "Buckle10g_" + ax + "_v3_r" + k;
        int err;
        CWStudy st = sm.CreateNewStudy3(name, 2, 0, out err);
        if (st == null) throw new Exception("study creation failed " + err);
        st.BucklingStudyOptions.BucklingModes = nModes;
        st.ContactManager.SetGlobalContact(0, 0);
        var lr = st.LoadsAndRestraintsManager;
        lr.AddRestraint(0, endZ0.ToArray(), null, out err); P("  fixture err " + err);
        var front = (Feature)((AssemblyDoc)doc).FeatureByName("Front Plane");
        CWGravity g = lr.AddGravity(front, out err);
        g.GravityBeginEdit();
        double a = -10 * 9.80665;
        g.SetGravitationalAcclerationValues(ax == 'X' ? a : 0, ax == 'Y' ? a : 0, ax == 'Z' ? a : 0);
        P("  gravity err " + err + "/" + g.GravityEndEdit());

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
        t0 = DateTime.Now;
        int re = st.RunAnalysis();
        P(string.Format("  solve err {0}, {1:F0} s", re, (DateTime.Now - t0).TotalSeconds));
        if (re != 0) { doc.Save3(1, ref e, ref w); return; }
        var res = st.Results;
        double[] blf = ToD(res.GetBucklingLoadFactors(out err));
        P("  raw load factors (err " + err + "): " + string.Join(", ", blf.Select(v => v.ToString("G5", ci)).ToArray()));
        for (int m = 1; m <= nModes; m++)
        {
            var d = ToD(res.GetMinMaxDisplacement(3, m, null, 0, out err));
            double x, y, z; st.Mesh.GetNodeLocation((int)d[2], out x, out y, out z);
            string hit = "";
            foreach (var c in comps)
            {
                var b = (double[])c.GetBox(false, false);
                if (b != null && x >= b[0] - 1e-4 && x <= b[3] + 1e-4 && y >= b[1] - 1e-4 && y <= b[4] + 1e-4 && z >= b[2] - 1e-4 && z <= b[5] + 1e-4)
                    hit += (hit == "" ? "" : "|") + c.Name2;
            }
            P(string.Format("  mode {0}: shape peak at ({1:F1}, {2:F1}, {3:F1}) mm in {4}", m, x * 1e3, y * 1e3, z * 1e3, hit));
        }
        doc.Save3(1, ref e, ref w);
        P("done " + DateTime.Now);
        log.Close();
    }
}
