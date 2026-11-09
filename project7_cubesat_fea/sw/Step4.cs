// Step 4: modal analysis of CubeSat_3U_v2 with two support cases.
//   cantilever : rail ends at Z = 0 fixed (vibration-fixture case, same as step 3)
//   deployer   : rail ends at Z = 0 and Z = 340.5 fixed + rail faces on rollers (no motion normal
//                to the rail faces, sliding allowed) = CubeSat held along its rails in a dispenser.
//                The X rail faces are part of the side-frame plate's outer face, so the ribs and end
//                bands on that face are also held (slightly stiff in X; noted in the README).
// Env: S4_BC = "cantilever,deployer" (default both), S4_H = 4, S4_N = 12 modes, S4_TAG.
// Output: results/step4_<tag>.txt with frequencies and effective mass fractions per mode.
using System;
using System.IO;
using System.Linq;
using System.Globalization;
using System.Collections.Generic;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.cosworks;

public static class Step4
{
    static readonly string Asm = @"D:\cad free projects\CubeSat 3U\" + (System.Environment.GetEnvironmentVariable("S4_ASM") ?? "CubeSat_3U_v2.SLDASM");
    static TextWriter log;
    static void P(string s) { Console.WriteLine(s); if (log != null) { log.WriteLine(s); log.Flush(); } }
    static double[] ToD(object o) { return ((Array)o).Cast<object>().Select(x => Convert.ToDouble(x)).ToArray(); }

    public static void Run(string outDir)
    {
        var ci = CultureInfo.InvariantCulture;
        string bcs = System.Environment.GetEnvironmentVariable("S4_BC") ?? "cantilever,deployer";
        double h = double.Parse(System.Environment.GetEnvironmentVariable("S4_H") ?? "4", ci);
        int nModes = int.Parse(System.Environment.GetEnvironmentVariable("S4_N") ?? "12");
        string tag = System.Environment.GetEnvironmentVariable("S4_TAG") ?? ("v2_h" + h.ToString(ci));
        log = new StreamWriter(Path.Combine(outDir, "step4_" + tag + ".txt"));
        P("Step 4 modal | BC " + bcs + " | h = " + h + " mm | " + nModes + " modes | " + DateTime.Now);

        var sw = SwSim.Connect(log);
        int e = 0, w = 0;
        var doc = (ModelDoc2)sw.GetOpenDocumentByName(Asm) ?? (ModelDoc2)sw.OpenDoc6(Asm, 2, 0, "", ref e, ref w);
        sw.ActivateDoc3(doc.GetTitle(), false, 0, ref e);
        var cw = SwSim.Simulation(sw, log);
        var sm = cw.ActiveDoc.StudyManager;
        var comps = ((object[])((AssemblyDoc)doc).GetComponents(true)).Cast<Component2>().ToList();
        double mass = ((MassProperty)doc.Extension.CreateMassProperty()).Mass;
        P(string.Format("components {0}, mass {1:F3} kg", comps.Count, mass));

        // side-frame faces, selected in part coordinates (frame 2 is frame 1 turned 180 deg about Z)
        var endZ0 = new List<object>(); var endZ1 = new List<object>();
        var railX = new List<object>(); var railY = new List<object>();
        foreach (var c in comps.Where(c => c.Name2.StartsWith("side_frame")))
            foreach (Body2 b in (object[])c.GetBodies2(0))
                foreach (Face2 f in (object[])b.GetFaces())
                {
                    if (!((Surface)f.GetSurface()).IsPlane()) continue;
                    double[] bx = (double[])f.GetBox();
                    Func<int, double, bool> on = (k, v) => Math.Abs(bx[k] - v) < 1e-7 && Math.Abs(bx[k + 3] - v) < 1e-7;
                    if (on(2, 0)) endZ0.Add(f);
                    else if (on(2, 0.3405)) endZ1.Add(f);
                    else if (on(0, 0)) railX.Add(f);                       // plate outer face (rail faces in X)
                    else if (on(1, 0) || on(1, 0.1)) railY.Add(f);         // flange outer faces (rail faces in Y)
                }
        P(string.Format("faces: ends Z0 {0}, Z1 {1}, rail X {2}, rail Y {3}", endZ0.Count, endZ1.Count, railX.Count, railY.Count));
        var dummies = comps.Where(c => c.Name2.StartsWith("camera") || c.Name2.StartsWith("rw_wheel")
                                    || c.Name2.StartsWith("battery_cell") || c.Name2.StartsWith("antenna")).Cast<object>().ToArray();
        var thin = comps.Where(c => c.Name2.StartsWith("shear_panel")).Cast<object>().ToArray();

        foreach (string bc in bcs.Split(','))
        {
            string name = "Modal_" + bc + "_" + tag;
            var existing = new HashSet<string>();
            for (int i = 0; i < sm.StudyCount; i++) { var s0 = sm.GetStudy(i); if (s0 != null) existing.Add(s0.Name); }
            for (int k = 2; existing.Contains(name); k++) name = "Modal_" + bc + "_" + tag + "_r" + k;
            int err;
            CWStudy st = sm.CreateNewStudy3(name, 1, 0, out err);
            if (st == null) throw new Exception("study creation failed " + err);
            st.FrequencyStudyOptions.NoOfFrequencies = nModes;
            st.ContactManager.SetGlobalContact(0, 0);                  // bonded
            var lr = st.LoadsAndRestraintsManager;
            lr.AddRestraint(0, endZ0.ToArray(), null, out err); P("  [" + bc + "] fixed Z0 err " + err);
            if (bc == "deployer")
            {
                lr.AddRestraint(0, endZ1.ToArray(), null, out err); P("  [" + bc + "] fixed Z1 err " + err);
                lr.AddRestraint(3, railX.ToArray(), null, out err); P("  [" + bc + "] roller rail X err " + err);
                lr.AddRestraint(3, railY.ToArray(), null, out err); P("  [" + bc + "] roller rail Y err " + err);
            }
            var mesh = st.Mesh;
            mesh.MesherType = 0; mesh.Quality = 1;
            var mc = mesh.ApplyMeshControl(dummies, out err);
            if (mc != null) { mc.Units = 0; mc.ElementSize = 3 * h; }
            var mc2 = mesh.ApplyMeshControl(thin, out err);
            if (mc2 != null) { mc2.Units = 0; mc2.ElementSize = Math.Min(h, 2.0); }
            var t0 = DateTime.Now;
            int me = st.CreateMesh(0, h, h * 0.05);
            P(string.Format("  [{0}] mesh err {1}, nodes {2}, {3:F0} s", bc, me, mesh.NodeCount, (DateTime.Now - t0).TotalSeconds));
            if (mesh.GetNoOfFailedComponents() > 0 || me != 0) throw new Exception("meshing failed");
            var t1 = DateTime.Now;
            int re = st.RunAnalysis();
            P(string.Format("  [{0}] solve err {1}, {2:F0} s", bc, re, (DateTime.Now - t1).TotalSeconds));
            if (re != 0) continue;
            var res = st.Results;
            double[] fr = ToD(res.GetResonantFrequencies(out err));      // [mode, rad/s, Hz, s] x n
            double[] mp = ToD(res.GetMassParticipation(out err));        // [mode, X, Y, Z] x n
            P("  [" + bc + "] mode   f (Hz)   eff. mass X     Y      Z");
            double sx = 0, sy = 0, sz = 0;
            for (int i = 0; 4 * i + 3 < fr.Length; i++)
            {
                double mx = 4 * i + 3 < mp.Length ? mp[4 * i + 1] : 0, my = 4 * i + 3 < mp.Length ? mp[4 * i + 2] : 0, mz = 4 * i + 3 < mp.Length ? mp[4 * i + 3] : 0;
                sx += mx; sy += my; sz += mz;
                P(string.Format("  [{0}] {1,4:F0} {2,9:F1}   {3,9:F4} {4,6:F4} {5,6:F4}", bc, fr[4 * i], fr[4 * i + 2], mx, my, mz));
            }
            P(string.Format("  [{0}] sum of effective mass: X {1:F3} Y {2:F3} Z {3:F3}", bc, sx, sy, sz));
            doc.Save3(1, ref e, ref w);
        }
        P("done " + DateTime.Now);
        log.Close();
    }
}
