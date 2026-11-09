// Design v3: X-braces in the three side-frame pockets (step-4 finding: whole-bus Y bending at
// 72 Hz, because the pocketed side frames are the shear webs for Y loads). Each pocket gets both
// corner-to-corner diagonals, 6 mm wide and 1.5 mm deep (= plate thickness, flush with both faces).
// Part coordinates: plate outer face x = 0, plate across y 0..100, length z 0..340.5;
// pockets y 8.5..91.5, z 12..109.5 / 117.5..223 / 231..328.5.
// Two thin-feature extrusions ("/" and "\" diagonals) because crossing lines in one thin sketch fail.
// Checks: volume must grow and the part box must not change (braces flush, nothing sticking out).
using System;
using System.Linq;
using SolidWorks.Interop.sldworks;

public static class Braces
{
    const string PartPath = @"D:\cad free projects\CubeSat 3U\side_frame.SLDPRT";
    const double W = 0.006, T = 0.0015;
    static readonly double[][] Pockets = { new[] { 0.012, 0.1095 }, new[] { 0.1175, 0.223 }, new[] { 0.231, 0.3285 } };
    const double Y0 = 0.0085, Y1 = 0.0915;

    public static void Run(string outDir)
    {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        int e = 0, w = 0;
        var open = (ModelDoc2)sw.GetOpenDocumentByName(PartPath);
        if (open != null && open.GetSaveFlag())
        {   // only accept leftovers of a failed run of this script; anything else = user's work, stop
            var lf = (Feature)((PartDoc)open).FeatureByName("Braces / sketch") ?? (Feature)((PartDoc)open).FeatureByName("Braces \\ sketch");
            if (lf == null) { Console.WriteLine("side_frame has UNSAVED changes - nothing done"); return; }
            open.ClearSelection2(true); lf.Select2(false, 0); open.EditDelete();
            Console.WriteLine("removed leftover sketch of a failed run; still dirty: " + open.GetSaveFlag());
        }
        var doc = open ?? (ModelDoc2)sw.OpenDoc6(PartPath, 1, 0, "", ref e, ref w);
        sw.ActivateDoc3(doc.GetTitle(), false, 0, ref e);
        var part = (PartDoc)doc;
        if (part.FeatureByName("Braces /") != null) { Console.WriteLine("braces already present - nothing done"); return; }
        var mu = (MathUtility)sw.GetMathUtility();
        double m0 = Mass(doc); double[] box0 = (double[])part.GetPartBox(true);
        Console.WriteLine(string.Format("before: {0:F3} g, box x {1:F2}..{2:F2} y {3:F2}..{4:F2} z {5:F2}..{6:F2} mm",
            m0 * 1e3, box0[0] * 1e3, box0[3] * 1e3, box0[1] * 1e3, box0[4] * 1e3, box0[2] * 1e3, box0[5] * 1e3));

        foreach (string dir in new[] { "/", "\\" })
        {
            // the plate's outer face: plane at x = 0 spanning the length
            Face2 face = null; double best = 0;
            foreach (Body2 b in (object[])part.GetBodies2(0, true))
                foreach (Face2 f in (object[])b.GetFaces())
                {
                    if (!((Surface)f.GetSurface()).IsPlane()) continue;
                    double[] bx = (double[])f.GetBox();
                    if (Math.Abs(bx[0]) < 1e-7 && Math.Abs(bx[3]) < 1e-7 && f.GetArea() > best) { best = f.GetArea(); face = f; }
                }
            if (face == null) throw new Exception("outer face x = 0 not found");
            doc.ClearSelection2(true);
            ((Entity)face).Select4(false, null);
            doc.SketchManager.InsertSketch(true);
            var sk = (Sketch)doc.SketchManager.ActiveSketch;
            var toSk = (MathTransform)sk.ModelToSketchTransform;
            Func<double, double, double[]> S = (y, z) => (double[])((MathPoint)((MathPoint)mu.CreatePoint(new[] { 0.0, y, z })).MultiplyTransform(toSk)).ArrayData;
            doc.SketchManager.AddToDB = true;
            foreach (var p in Pockets)
            {
                double[] a = S(dir == "/" ? Y0 : Y1, p[0]), c = S(dir == "/" ? Y1 : Y0, p[1]);
                var ln = (SketchSegment)doc.SketchManager.CreateLine(a[0], a[1], 0, c[0], c[1], 0);
                doc.ClearSelection2(true); ln.Select4(false, null); doc.SketchAddConstraints("sgFIXED");
            }
            doc.SketchManager.AddToDB = false;
            doc.ClearSelection2(true);
            doc.SketchManager.InsertSketch(true);
            var skf = (Feature)sk; skf.Name = "Braces " + dir + " sketch";
            Feature ft = null;
            double v0 = Vol(doc);
            foreach (bool flip in new[] { true, false })
            {
                doc.ClearSelection2(true); skf.Select2(false, 0);
                // thin wall type 2 = mid-plane
                ft = (Feature)doc.FeatureManager.FeatureExtrusionThin2(true, false, flip, 0, 0, T, 0,
                    false, false, false, false, 0, 0, false, false, false, false, true,
                    W, W, 0, 2, 0, false, 0, true, true, 0, 0, false);
                double[] bx = (double[])part.GetPartBox(true);
                Console.WriteLine(string.Format("  flip {0}: feature {1}, dV {2:F1} mm3, box x {3:F2}..{4:F2}", flip, ft != null,
                    (Vol(doc) - v0) * 1e9, bx[0] * 1e3, bx[3] * 1e3));
                bool same = Enumerable.Range(0, 6).All(i => Math.Abs(bx[i] - box0[i]) < 1e-7);
                if (ft != null && same && Vol(doc) > v0) break;
                if (ft != null) { doc.EditUndo2(1); ft = null; }
            }
            if (ft == null) throw new Exception("brace extrusion " + dir + " failed - part NOT saved");
            ft.Name = "Braces " + dir;
            Console.WriteLine(string.Format("Braces {0}: +{1:F3} g", dir, (Vol(doc) - v0) * 2810 * 1e3));
        }
        double m1 = Mass(doc);
        Console.WriteLine(string.Format("after: {0:F3} g (+{1:F3} g)", m1 * 1e3, (m1 - m0) * 1e3));
        if (m1 - m0 < 0.012 || m1 - m0 > 0.022) throw new Exception("unexpected mass change - part NOT saved");
        doc.Save3(1, ref e, ref w);
        Console.WriteLine("saved (errors " + e + ", warnings " + w + ")");
    }
    static double Mass(ModelDoc2 d) { return ((MassProperty)d.Extension.CreateMassProperty()).Mass; }
    static double Vol(ModelDoc2 d) { return ((MassProperty)d.Extension.CreateMassProperty()).Volume; }
}
