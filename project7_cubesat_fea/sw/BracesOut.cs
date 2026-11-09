// Design v4: thicken the X-braces outward (step-B finding: without help from the bonded solar panels the
// pocket-1 braces buckle out of plane at BLF 5.7 < 10.6). Adds, on the plate's outer face (x = 0), 1.5 mm
// outward: the same diagonals (6 mm wide) -> braces 3 mm thick in total (bending stiffness out of plane x8),
// plus 1.5 mm pads on the end bands and the 1U ribs (y 8.5..91.5) so the solar panels still sit flat (x = -1.5).
// Rail strips (y < 8.5, y > 91.5) are untouched. Checks: mass change and new part box x min = -1.5 mm.
using System;
using System.Linq;
using SolidWorks.Interop.sldworks;

public static class BracesOut
{
    const string PartPath = @"D:\cad free projects\CubeSat 3U\side_frame.SLDPRT";
    const double W = 0.006, T = 0.0015, Y0 = 0.0085, Y1 = 0.0915;
    static readonly double[][] Pockets = { new[] { 0.012, 0.1095 }, new[] { 0.1175, 0.223 }, new[] { 0.231, 0.3285 } };
    static readonly double[][] Pads = { new[] { 0.004, 0.012 }, new[] { 0.1095, 0.1175 }, new[] { 0.223, 0.231 }, new[] { 0.3285, 0.3365 } };

    public static void Run(string outDir)
    {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        int e = 0, w = 0;
        var open = (ModelDoc2)sw.GetOpenDocumentByName(PartPath);
        if (open != null && open.GetSaveFlag()) { Console.WriteLine("side_frame has UNSAVED changes - nothing done"); return; }
        var doc = open ?? (ModelDoc2)sw.OpenDoc6(PartPath, 1, 0, "", ref e, ref w);
        sw.ActivateDoc3(doc.GetTitle(), false, 0, ref e);
        var part = (PartDoc)doc;
        if (part.FeatureByName("Braces / outer") != null) { Console.WriteLine("already done"); return; }
        if (part.FeatureByName("Braces /") == null) throw new Exception("v3 braces missing");
        var mu = (MathUtility)sw.GetMathUtility();
        double m0 = Mass(doc);

        foreach (string what in new[] { "/", "\\", "pads" })
        {
            Face2 face = null; double best = 0;
            foreach (Body2 b in (object[])part.GetBodies2(0, true))
                foreach (Face2 f in (object[])b.GetFaces())
                {
                    if (!((Surface)f.GetSurface()).IsPlane()) continue;
                    double[] bx = (double[])f.GetBox();
                    if (Math.Abs(bx[0]) < 1e-7 && Math.Abs(bx[3]) < 1e-7 && f.GetArea() > best) { best = f.GetArea(); face = f; }
                }
            doc.ClearSelection2(true);
            ((Entity)face).Select4(false, null);
            doc.SketchManager.InsertSketch(true);
            var sk = (Sketch)doc.SketchManager.ActiveSketch;
            var toSk = (MathTransform)sk.ModelToSketchTransform;
            Func<double, double, double[]> S = (y, z) => (double[])((MathPoint)((MathPoint)mu.CreatePoint(new[] { 0.0, y, z })).MultiplyTransform(toSk)).ArrayData;
            doc.SketchManager.AddToDB = true;
            if (what == "pads")
                foreach (var p in Pads)
                {
                    double[] a = S(Y0, p[0]), c = S(Y1, p[1]);
                    doc.SketchManager.CreateCornerRectangle(a[0], a[1], 0, c[0], c[1], 0);
                }
            else
                foreach (var p in Pockets)
                {
                    double[] a = S(what == "/" ? Y0 : Y1, p[0]), c = S(what == "/" ? Y1 : Y0, p[1]);
                    doc.SketchManager.CreateLine(a[0], a[1], 0, c[0], c[1], 0);
                }
            doc.SketchManager.AddToDB = false;
            doc.ClearSelection2(true);
            doc.SketchManager.InsertSketch(true);
            var skf = (Feature)sk;
            string nm = what == "pads" ? "Panel pads" : "Braces " + what + " outer";
            skf.Name = nm + " sketch";
            double v0 = Vol(doc); Feature ft = null;
            foreach (bool dir in new[] { false, true })
            {
                doc.ClearSelection2(true); skf.Select2(false, 0);
                if (what == "pads")
                    ft = (Feature)doc.FeatureManager.FeatureExtrusion3(true, false, dir, 0, 0, T, 0, false, false, false, false,
                        0, 0, false, false, false, false, true, true, true, 0, 0, false);
                else
                    ft = (Feature)doc.FeatureManager.FeatureExtrusionThin2(true, false, dir, 0, 0, T, 0,
                        false, false, false, false, 0, 0, false, false, false, false, true,
                        W, W, 0, 2, 0, false, 0, true, true, 0, 0, false);
                double[] bx = (double[])part.GetPartBox(true);
                Console.WriteLine(string.Format("  {0} dir {1}: feature {2}, dV {3:F1} mm3, x min {4:F2}", nm, dir, ft != null, (Vol(doc) - v0) * 1e9, bx[0] * 1e3));
                if (ft != null && Math.Abs(bx[0] + T) < 1e-7 && Vol(doc) > v0) break;
                if (ft != null) { doc.EditUndo2(1); ft = null; }
            }
            if (ft == null) throw new Exception(nm + " failed - part NOT saved");
            ft.Name = nm;
        }
        double m1 = Mass(doc);
        Console.WriteLine(string.Format("side frame {0:F3} -> {1:F3} g (+{2:F3})", m0 * 1e3, m1 * 1e3, (m1 - m0) * 1e3));
        if (m1 - m0 < 0.020 || m1 - m0 > 0.040) throw new Exception("unexpected mass change - part NOT saved");
        doc.Save3(1, ref e, ref w);
        Console.WriteLine("saved (errors " + e + ")");
    }
    static double Mass(ModelDoc2 d) { return ((MassProperty)d.Extension.CreateMassProperty()).Mass; }
    static double Vol(ModelDoc2 d) { return ((MassProperty)d.Extension.CreateMassProperty()).Volume; }
}
