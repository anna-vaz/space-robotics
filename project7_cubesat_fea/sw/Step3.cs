// Step 3: quasi-static 10 g FEA of the full CubeSat (CubeSat_3U.SLDASM), SOLIDWORKS Simulation.
//
// Environment variables
//   S3_AXES   which load directions to solve, e.g. "XYZ" (default) or "Z"
//   S3_H      global element size in mm (default 3)
//   S3_BC     "cantilever" = rail ends at Z = 0 fixed (vibration-fixture case, compares with the
//             step-1 hand calcs) or "deployer" = rail ends at both Z = 0 and Z = 340.5 fixed
//   S3_TAG    suffix for study names / result files
// Contacts: global bonded (screws not modelled). Mass dummies get a coarser mesh control.
// Output: results/step3_<tag>.txt (also on stdout) + plot images in results/fea/.
using System;
using System.IO;
using System.Linq;
using System.Globalization;
using System.Collections.Generic;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.cosworks;

public static class Step3
{
    static readonly string Asm = @"D:\cad free projects\CubeSat 3U\" + (System.Environment.GetEnvironmentVariable("S3_ASM") ?? "CubeSat_3U_v2.SLDASM");
    static TextWriter log;

    static void P(string s) { Console.WriteLine(s); if (log != null) { log.WriteLine(s); log.Flush(); } }

    public static void Run(string outDir)
    {
        var ci = CultureInfo.InvariantCulture;
        string axes = System.Environment.GetEnvironmentVariable("S3_AXES") ?? "XYZ";
        double h = double.Parse(System.Environment.GetEnvironmentVariable("S3_H") ?? "3", ci);
        string bc = System.Environment.GetEnvironmentVariable("S3_BC") ?? "cantilever";
        string tag = System.Environment.GetEnvironmentVariable("S3_TAG") ?? (bc + "_h" + h.ToString(ci));
        string feaDir = Path.Combine(outDir, "fea");
        Directory.CreateDirectory(feaDir);
        log = new StreamWriter(Path.Combine(outDir, "step3_" + tag + ".txt"));
        P("Step 3 static 10 g | axes " + axes + " | h = " + h + " mm | BC " + bc + " | " + DateTime.Now);

        var sw = SwSim.Connect(log);
        int e = 0, w = 0;
        // free memory before meshing; save first so SOLIDWORKS doesn't ask "save Simulation data?"
        // Close nothing if the assembly is already open: closing ANY document (even its own component
        // parts) makes Simulation pop up a modal "save Simulation data?" box that blocks the script.
        if (sw.GetOpenDocumentByName(Asm) == null)
            foreach (ModelDoc2 od in ((object[])sw.GetDocuments() ?? new object[0]).Cast<ModelDoc2>().ToList())
            {
                if (od.GetSaveFlag()) od.Save3(1, ref e, ref w);
                sw.CloseDoc(od.GetTitle());
            }
        var doc = (ModelDoc2)sw.OpenDoc6(Asm, 2, 0, "", ref e, ref w);
        sw.ActivateDoc3(doc.GetTitle(), false, 0, ref e);
        var cw = SwSim.Simulation(sw, log);
        var sm = cw.ActiveDoc.StudyManager;
        var comps = ((object[])((AssemblyDoc)doc).GetComponents(true)).Cast<Component2>().ToList();
        double mass = ((MassProperty)doc.Extension.CreateMassProperty()).Mass;
        P(string.Format("components {0}, mass {1:F3} kg -> 10 g load {2:F1} N", comps.Count, mass, mass * 98.0665));

        // rail-end faces (planar faces of the side frames lying in Z = 0 [and Z = 340.5])
        var fixedFaces = new List<object>();
        foreach (var c in comps.Where(c => c.Name2.StartsWith("side_frame")))
            foreach (Body2 b in (object[])c.GetBodies2(0))
                foreach (Face2 f in (object[])b.GetFaces())
                {
                    if (!((Surface)f.GetSurface()).IsPlane()) continue;
                    double[] bx = (double[])f.GetBox();            // component frame = world Z for frames
                    bool z0 = Math.Abs(bx[2]) < 1e-7 && Math.Abs(bx[5]) < 1e-7;
                    bool z1 = Math.Abs(bx[2] - 0.3405) < 1e-7 && Math.Abs(bx[5] - 0.3405) < 1e-7;
                    if (z0 || (bc == "deployer" && z1)) fixedFaces.Add(f);
                }
        P("fixed rail-end faces: " + fixedFaces.Count);
        var dummies = comps.Where(c => c.Name2.StartsWith("camera") || c.Name2.StartsWith("rw_wheel")
                                    || c.Name2.StartsWith("battery_cell") || c.Name2.StartsWith("antenna")).Cast<object>().ToArray();

        // ONE study for all load directions: only the gravity vector changes between solves. A second
        // study with global bonding pops up a modal "interaction entity cannot be processed" warning,
        // and DeleteStudy pops up "save Simulation data?" - both block the script.
        string name = "QS10g_" + tag;
        var existing = new HashSet<string>();
        for (int i = 0; i < sm.StudyCount; i++) { var s0 = sm.GetStudy(i); if (s0 != null) existing.Add(s0.Name); }
        for (int k = 2; existing.Contains(name); k++) name = "QS10g_" + tag + "_r" + k;
        int err;
        CWStudy st = sm.CreateNewStudy3(name, 0, 0, out err);
        if (st == null) throw new Exception("study creation failed " + err);
        st.ContactManager.SetGlobalContact(0, 0);                  // bonded
        var lr = st.LoadsAndRestraintsManager;
        lr.AddRestraint(0, fixedFaces.ToArray(), null, out err);
        P("  fixture err " + err);
        // gravity; reference = assembly Front Plane (dir1 = X, dir2 = Y, normal = Z)
        var front = ((Feature)((AssemblyDoc)doc).FeatureByName("Front Plane"));
        CWGravity g = lr.AddGravity(front, out err);
        P("  gravity err " + err);

        var mesh = st.Mesh;
        string mt = System.Environment.GetEnvironmentVariable("S3_MESHER") ?? "0";
        mesh.MesherType = int.Parse(mt); mesh.Quality = 1;
        if (mesh.MesherType != 0) { mesh.Unit = 0; mesh.MaxElementSize = h; mesh.MinElementSize = h / 4; }
        P("  mesher type " + mesh.MesherType);
        var mc = mesh.ApplyMeshControl(dummies, out err);
        if (mc != null) { mc.Units = 0; mc.ElementSize = 3 * h; }
        P("  mesh control on " + dummies.Length + " dummies (" + 3 * h + " mm), err " + err);
        // 1 mm shear panels fail to mesh with coarse elements: cap their size at 2 mm
        var thin = comps.Where(c => c.Name2.StartsWith("shear_panel")).Cast<object>().ToArray();
        var mc2 = mesh.ApplyMeshControl(thin, out err);
        if (mc2 != null) { mc2.Units = 0; mc2.ElementSize = Math.Min(h, 2.0); }
        P("  mesh control on shear panels (" + Math.Min(h, 2.0) + " mm), err " + err);
        var t0 = DateTime.Now;
        int me = st.CreateMesh(0, h, h * 0.05);
        P(string.Format("  mesh: err {0}, nodes {1}, elements {2}, {3:F0} s", me, mesh.NodeCount, mesh.ElementCount, (DateTime.Now - t0).TotalSeconds));
        int nf = mesh.GetNoOfFailedComponents();
        if (nf > 0)
        {
            int n; var fc = (object[])mesh.GetFailedComponents(out n);
            throw new Exception("mesh failed for: " + string.Join(", ", fc.Select(o => o is Component2 ? ((Component2)o).Name2 : o.ToString()).ToArray()));
        }
        if (me != 0) throw new Exception("meshing failed");

        foreach (char ax in axes)
        {
            g.GravityBeginEdit();
            double a = -10 * 9.80665;
            g.SetGravitationalAcclerationValues(ax == 'X' ? a : 0, ax == 'Y' ? a : 0, ax == 'Z' ? a : 0);
            P("  [" + ax + "] gravity set, err " + g.GravityEndEdit());
            var t1 = DateTime.Now;
            int re = st.RunAnalysis();
            P(string.Format("  [{0}] solve err {1}, {2:F0} s", ax, re, (DateTime.Now - t1).TotalSeconds));
            if (re != 0) continue;
            Report(st, comps, ax, feaDir, doc, tag);
        }
        doc.Save3(1, ref e, ref w);
        P("done " + DateTime.Now);
        log.Close();
    }

    static double[] ToD(object o) { return ((Array)o).Cast<object>().Select(x => Convert.ToDouble(x)).ToArray(); }

    static void Report(CWStudy st, List<Component2> comps, char ax, string feaDir, ModelDoc2 doc, string tag)
    {
        int err;
        var res = st.Results;
        double[] d = ToD(res.GetMinMaxDisplacement(3, 1, null, 0, out err));           // URES, mm
        P(string.Format("  [{0}] max displacement {1:F4} mm (node {2})", ax, d[3], d[2]));
        double[] s = ToD(res.GetMinMaxStress(9, 0, 1, null, 3, out err));               // von Mises, MPa
        P(string.Format("  [{0}] max von Mises {1:F2} MPa (node {2})", ax, s[3], s[2]));
        double nx, ny, nz;
        st.Mesh.GetNodeLocation((int)d[2], out nx, out ny, out nz);
        P(string.Format("  [{0}] max-displacement node at ({1:F1}, {2:F1}, {3:F1}) mm", ax, nx * 1e3, ny * 1e3, nz * 1e3));
        st.Mesh.GetNodeLocation((int)s[2], out nx, out ny, out nz);
        P(string.Format("  [{0}] max-stress node at ({1:F1}, {2:F1}, {3:F1}) mm", ax, nx * 1e3, ny * 1e3, nz * 1e3));
        // reactions come per node: [node, Fx, Fy, Fz, Fres, Mx, My, Mz, Mres] repeated -> sum
        double[] rf = ToD(res.GetReactionForcesAndMoments(1, null, 0, out err));
        double fx = 0, fy = 0, fz = 0;
        for (int i = 0; i + 8 < rf.Length; i += 9) { fx += rf[i + 1]; fy += rf[i + 2]; fz += rf[i + 3]; }
        P(string.Format("  [{0}] reaction sum (N): Fx {1:F2} Fy {2:F2} Fz {3:F2} |F| {4:F2}  ({5} entries)",
            ax, fx, fy, fz, Math.Sqrt(fx * fx + fy * fy + fz * fz), rf.Length / 9));
        // per component: max nodal von Mises on its faces
        var groups = new Dictionary<string, double>();
        var disp = new Dictionary<string, double>();
        foreach (var c in comps)
        {
            var faces = new List<object>();
            foreach (Body2 b in (object[])c.GetBodies2(0)) faces.AddRange((object[])b.GetFaces());
            try
            {
                object v = res.GetStressForEntities2(true, 9, 1, null, faces.ToArray(), 3, out err);
                if (v == null) continue;
                if (v is DBNull) continue;
                double[] pv = ToD(v);                      // [node, value] pairs
                double mx = 0;
                for (int i = 1; i < pv.Length; i += 2) mx = Math.Max(mx, pv[i]);
                string key = c.Name2.Substring(0, c.Name2.LastIndexOf('-'));
                if (!groups.ContainsKey(key) || groups[key] < mx) groups[key] = mx;
                object dv = res.GetDisplacementForEntities(3, 1, null, faces.ToArray(), 0, out err);
                if (!(dv is DBNull) && dv != null)
                {
                    double[] dd = ToD(dv); double dm = 0;
                    for (int i = 1; i < dd.Length; i += 2) dm = Math.Max(dm, dd[i]);
                    if (!disp.ContainsKey(key) || disp[key] < dm) disp[key] = dm;
                }
            }
            catch (Exception ex) { P("    stress read failed for " + c.Name2 + ": " + ex.Message); }
        }
        foreach (var kv in groups.OrderByDescending(k => k.Value))
            P(string.Format("    {0,-18} max vM {1,8:F2} MPa   max disp {2,7:F4} mm", kv.Key, kv.Value,
                disp.ContainsKey(kv.Key) ? disp[kv.Key] : double.NaN));
        foreach (var plot in new[] { new[] { 1, 9, 3 }, new[] { 0, 3, 0 } })
        {
            CWPlot p = res.CreatePlot(plot[0], plot[1], plot[2], false, out err);
            if (p == null) { P("  plot failed " + err); continue; }
            p.ActivatePlot();
            doc.ShowNamedView2("*Trimetric", -1);
            doc.ViewZoomtofit2();
            doc.GraphicsRedraw2();
            string f = Path.Combine(feaDir, "step3_" + tag + "_" + ax + (plot[0] == 1 ? "_vonmises" : "_disp") + ".bmp");
            doc.SaveBMP(f, 1600, 1200);
        }
    }
}
