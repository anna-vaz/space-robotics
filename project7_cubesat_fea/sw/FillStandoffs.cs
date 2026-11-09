// Fills the bore of every standoff tube (the M3 screw that sits in it is not modelled), turning
// the 0.9 mm-wall tubes into solid 5 mm rods. Removes the 0.1 mm contact ring (bore r1.6 vs board
// hole r1.7) that the mesher could not handle. Edits the parts in place; closes nothing (closing
// documents makes Simulation pop up a modal dialog).
using System;
using System.IO;
using System.Linq;
using SolidWorks.Interop.sldworks;

public static class FillStandoffs
{
    const string Dir = @"D:\cad free projects\CubeSat 3U\";

    public static void Run(string outDir)
    {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        int e = 0, w = 0;
        var asm = (ModelDoc2)sw.GetOpenDocumentByName(Dir + "CubeSat_3U.SLDASM");
        double m0 = asm == null ? 0 : ((MassProperty)asm.Extension.CreateMassProperty()).Mass;
        foreach (string f in Directory.GetFiles(Dir, "standoff_*.SLDPRT"))
        {
            var doc = (ModelDoc2)sw.GetOpenDocumentByName(f) ?? (ModelDoc2)sw.OpenDoc6(f, 1, 1, "", ref e, ref w);
            sw.ActivateDoc3(doc.GetTitle(), false, 0, ref e);
            var part = (PartDoc)doc;
            if (part.FeatureByName("Bore fill (screw)") != null) { Console.WriteLine(Path.GetFileName(f) + ": already filled"); continue; }
            double[] box = (double[])part.GetPartBox(true);
            double L = box[5] - box[2];
            double v0 = ((MassProperty)doc.Extension.CreateMassProperty()).Volume;
            doc.ClearSelection2(true);
            doc.Extension.SelectByID2("Front Plane", "PLANE", 0, 0, 0, false, 0, null, 0);
            doc.SketchManager.InsertSketch(true);
            doc.SketchManager.AddToDB = true;
            var c = (SketchSegment)doc.SketchManager.CreateCircleByRadius(0, 0, 0, 0.0016);
            doc.SketchManager.AddToDB = false;
            doc.ClearSelection2(true);
            c.Select4(false, null);
            doc.SketchAddConstraints("sgFIXED");
            doc.ClearSelection2(true);
            var sk = (Feature)doc.SketchManager.ActiveSketch;
            doc.SketchManager.InsertSketch(true);
            doc.ClearSelection2(true);
            sk.Select2(false, 0);
            var ft = (Feature)doc.FeatureManager.FeatureExtrusion2(true, false, false, 0, 0, L, 0,
                false, false, false, false, 0, 0, false, false, false, false, true, false, true, 0, 0, false);
            if (ft == null) throw new Exception("fill failed for " + f);
            ft.Name = "Bore fill (screw)"; sk.Name = "Bore fill sketch";
            double v1 = ((MassProperty)doc.Extension.CreateMassProperty()).Volume;
            doc.Save3(1, ref e, ref w);
            Console.WriteLine(string.Format("{0}: L {1:F1} mm, volume {2:F1} -> {3:F1} mm^3", Path.GetFileName(f), L * 1e3, v0 * 1e9, v1 * 1e9));
        }
        if (asm != null)
        {
            sw.ActivateDoc3(asm.GetTitle(), false, 0, ref e);
            asm.EditRebuild3();
            double m1 = ((MassProperty)asm.Extension.CreateMassProperty()).Mass;
            asm.Save3(1, ref e, ref w);
            Console.WriteLine(string.Format("assembly mass {0:F2} -> {1:F2} g", m0 * 1e3, m1 * 1e3));
        }
    }
}
