// Builds the first assembly of the 3U structure: 2 side frames, 2 shear panels, 2 end plates.
// Box coordinates: X normal to the side-frame plates, Y normal to the shear panels, Z along
// the rails (0 .. 340.5 mm). Every part has its origin at a corner, so each placement is an
// exact rigid transform; components are then fixed (no mates).
// Checks: mass + CoM, interference detection (touching faces allowed), and that every screw
// hole is coaxial with its partner hole in the mating part.
using System;
using System.IO;
using System.Collections.Generic;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;

public static class Assembly
{
    const string Dir = @"D:\cad free projects\CubeSat 3U\";
    const string AsmPath = Dir + "CubeSat_3U_structure.SLDASM";

    // name, file, 180 deg about Z?, translation (mm)
    static readonly object[][] Placements = {
        new object[] {"side frame -X", "side_frame.SLDPRT", false, 0.0, 0.0, 0.0},
        new object[] {"side frame +X", "side_frame.SLDPRT", true, 100.0, 100.0, 0.0},
        new object[] {"shear panel -Y", "shear_panel.SLDPRT", false, 1.5, 1.5, 7.0},
        new object[] {"shear panel +Y", "shear_panel.SLDPRT", false, 1.5, 97.5, 7.0},
        new object[] {"end plate -Z", "end_plate.SLDPRT", false, 1.5, 2.5, 7.0},
        new object[] {"end plate +Z", "end_plate.SLDPRT", false, 1.5, 2.5, 331.5},
    };

    public static void Run(string outDir)
    {
        if (File.Exists(AsmPath)) { Console.WriteLine("assembly already exists - nothing done"); return; }
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        sw.Visible = true;
        int e = 0, w = 0;
        foreach (string f in new[] { "side_frame.SLDPRT", "shear_panel.SLDPRT", "end_plate.SLDPRT" })
        {
            var d = (ModelDoc2)sw.GetOpenDocumentByName(Dir + f);
            if (d != null && d.GetSaveFlag()) throw new Exception(f + " has unsaved changes - nothing done");
            if (d == null) sw.OpenDoc6(Dir + f, 1, 1, "", ref e, ref w);   // silent, needed by AddComponent
        }
        var doc = (ModelDoc2)sw.NewDocument(sw.GetUserPreferenceStringValue(9), 0, 0, 0);
        doc.Extension.SetUserPreferenceInteger((int)swUserPreferenceIntegerValue_e.swUnitSystem, 0,
                                               (int)swUnitSystem_e.swUnitSystem_MMGS);
        var asm = (AssemblyDoc)doc;
        var mu = (MathUtility)sw.GetMathUtility();
        var comps = new List<Component2>();
        foreach (var p in Placements)
        {
            var c = (Component2)asm.AddComponent5(Dir + (string)p[1], 0, "", false, "", 0, 0, 0);
            if (c == null) throw new Exception("could not add " + p[1]);
            double s = (bool)p[2] ? -1 : 1;
            var t = (MathTransform)mu.CreateTransform(new double[] {
                s, 0, 0,  0, s, 0,  0, 0, 1,
                (double)p[3] / 1000, (double)p[4] / 1000, (double)p[5] / 1000,  1, 0, 0, 0 });
            c.Transform2 = t;
            comps.Add(c);
            Console.WriteLine("placed " + c.Name2 + " = " + p[0]);
        }
        doc.ClearSelection2(true);
        foreach (var c in comps) c.Select4(true, null, false);
        asm.FixComponent();
        doc.ClearSelection2(true);
        doc.EditRebuild3();

        // ---- mass properties
        var mp = (MassProperty)doc.Extension.CreateMassProperty();
        double[] cm = (double[])mp.CenterOfMass;
        Console.WriteLine(string.Format("assembly mass {0:F2} g (expected 393.39), CoM ({1:F3}, {2:F3}, {3:F3}) mm",
            mp.Mass * 1e3, cm[0] * 1e3, cm[1] * 1e3, cm[2] * 1e3));
        double[] mom = (double[])mp.GetMomentOfInertia(0);   // about CoM, assembly axes
        Console.WriteLine(string.Format("inertia about CoM: Ixx {0:E3} Iyy {1:E3} Izz {2:E3} kg m^2", mom[0], mom[4], mom[8]));

        // ---- interference (coincident faces are not counted)
        var idm = asm.InterferenceDetectionManager;
        idm.TreatCoincidenceAsInterference = false;
        idm.TreatSubAssembliesAsComponents = true;
        idm.IncludeMultibodyPartInterferences = true;
        object[] intf = (object[])idm.GetInterferences();
        int nI = intf == null ? 0 : intf.Length;
        Console.WriteLine("interferences: " + nI);
        if (intf != null)
            foreach (IInterference it in intf)
            {
                string names = "";
                foreach (Component2 c in (object[])it.Components) names += c.Name2 + " ";
                Console.WriteLine(string.Format("  {0}: volume {1:F3} mm^3", names, it.Volume * 1e9));
            }
        idm.Done();

        // ---- hole alignment: world axes of every screw/tap hole
        var holes = new List<double[]>();   // x, y, z, axis index, radius, component index
        for (int ci = 0; ci < comps.Count; ci++)
        {
            var c = comps[ci];
            var tr = (MathTransform)c.Transform2;
            foreach (Body2 b in (object[])c.GetBodies2(0))
                foreach (Face2 f in (object[])b.GetFaces())
                {
                    var su = (Surface)f.GetSurface();
                    if (!su.IsCylinder()) continue;
                    double[] q = (double[])su.CylinderParams;
                    double r = q[6];
                    if (r > 0.0018) continue;            // only screw holes (r 0.8 / 1.2 / 1.7)
                    if (Math.Abs(r - 0.001) < 1e-7 || Math.Abs(r - 0.003) < 1e-7) continue; // fillets
                    var pt = (MathPoint)((MathPoint)mu.CreatePoint(new double[] { q[0], q[1], q[2] })).MultiplyTransform(tr);
                    var ax = (MathVector)((MathVector)mu.CreateVector(new double[] { q[3], q[4], q[5] })).MultiplyTransform(tr);
                    double[] P = (double[])pt.ArrayData, A = (double[])ax.ArrayData;
                    int k = Math.Abs(A[0]) > 0.9 ? 0 : Math.Abs(A[1]) > 0.9 ? 1 : 2;
                    holes.Add(new double[] { P[0], P[1], P[2], k, r, ci });
                }
        }
        // pair: tap (r 0.8) with clearance (r 1.2) on the same axis line
        int paired = 0, taps = 0;
        foreach (var h in holes)
        {
            if (Math.Abs(h[4] - 0.0008) > 1e-7) continue;
            taps++;
            int k = (int)h[3];
            double best = 1e9; string partner = "none";
            foreach (var g in holes)
            {
                if (Math.Abs(g[4] - 0.0012) > 1e-7 || (int)g[3] != k) continue;
                double d = 0;
                for (int i = 0; i < 3; i++) if (i != k) d += (h[i] - g[i]) * (h[i] - g[i]);
                d = Math.Sqrt(d);
                if (d < best) { best = d; partner = comps[(int)g[5]].Name2; }
            }
            if (best < 1e-6) paired++;
            Console.WriteLine(string.Format("  tap in {0,-16} at ({1,7:F3},{2,7:F3},{3,8:F3}) axis {4} -> {5} offset {6:F4} mm",
                comps[(int)h[5]].Name2, h[0] * 1e3, h[1] * 1e3, h[2] * 1e3, "XYZ"[k], partner, best * 1e3));
        }
        Console.WriteLine("tap holes coaxial with a clearance hole: " + paired + " / " + taps);

        doc.ShowNamedView2("*Trimetric", -1);
        doc.ViewZoomtofit2();
        doc.Extension.SaveAs(AsmPath, 0, 1, null, ref e, ref w);
        Console.WriteLine("saved " + AsmPath + " (errors " + e + ", warnings " + w + ")");
    }
}
