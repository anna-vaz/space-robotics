// Read-only inspection of the active SOLIDWORKS part: mass, volume, box,
// material, features with their dimensions. Never modifies or saves the model.
using System;
using System.IO;
using SolidWorks.Interop.sldworks;

public static class Inspect
{
    public static void Run(string outDir)
    {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        foreach (object o in (object[])sw.GetDocuments()) { Report((ModelDoc2)o); Console.WriteLine(); }
    }
    static void Report(ModelDoc2 doc)
    {
        if (doc == null) { Console.WriteLine("no active document"); return; }
        Console.WriteLine("doc: " + doc.GetTitle() + "  path: " + doc.GetPathName() + "  type " + doc.GetType());
        Console.WriteLine("units (0=mm..): " + doc.LengthUnit);
        var part = doc as PartDoc;
        if (part != null)
        {
            string db;
            Console.WriteLine("material: " + part.GetMaterialPropertyName2("", out db) + " [" + db + "]");
            object[] bodies = (object[])part.GetBodies2(0, true);
            Console.WriteLine("solid bodies: " + (bodies == null ? 0 : bodies.Length));
            double[] box = (double[])part.GetPartBox(true);
            Console.WriteLine(string.Format("bounding box (mm): x {0:F3}..{1:F3}  y {2:F3}..{3:F3}  z {4:F3}..{5:F3}",
                box[0]*1e3, box[3]*1e3, box[1]*1e3, box[4]*1e3, box[2]*1e3, box[5]*1e3));
        }
        var mp = (MassProperty)doc.Extension.CreateMassProperty();
        Console.WriteLine(string.Format("mass {0:F2} g   volume {1:F1} mm^3   density {2:F0} kg/m^3",
            mp.Mass * 1e3, mp.Volume * 1e9, mp.Density));
        double[] c = (double[])mp.CenterOfMass;
        Console.WriteLine(string.Format("centre of mass (mm): {0:F3} {1:F3} {2:F3}", c[0]*1e3, c[1]*1e3, c[2]*1e3));

        Console.WriteLine("== cylindrical faces (hole centres, mm)");
        if (doc is PartDoc)
            foreach (Body2 b in (object[])((PartDoc)doc).GetBodies2(0, true))
                foreach (Face2 fc in (object[])b.GetFaces())
                {
                    var s = (Surface)fc.GetSurface();
                    if (!s.IsCylinder()) continue;
                    double[] p = (double[])s.CylinderParams;
                    Console.WriteLine(string.Format("  r {0:F3}  origin ({1:F3}, {2:F3}, {3:F3})  axis ({4:F0},{5:F0},{6:F0})",
                        p[6]*1e3, p[0]*1e3, p[1]*1e3, p[2]*1e3, p[3], p[4], p[5]));
                }
        Console.WriteLine("== features");
        var f = (Feature)doc.FirstFeature();
        while (f != null)
        {
            string t = f.GetTypeName2();
            if (t != "MaterialFolder" && !t.EndsWith("Folder") && t != "HistoryFolder")
            {
                Console.WriteLine("  " + f.Name + "  [" + t + "]" + (f.IsSuppressed() ? " SUPPRESSED" : ""));
                var dd = (DisplayDimension)f.GetFirstDisplayDimension();
                while (dd != null)
                {
                    var d = (Dimension)dd.GetDimension2(0);
                    Console.WriteLine(string.Format("      {0} = {1:F4}", d.FullName, d.SystemValue * (d.GetType() == 3 ? 57.29578 : 1e3)));
                    dd = (DisplayDimension)f.GetNextDisplayDimension(dd);
                }
                var sub = (Feature)f.GetFirstSubFeature();
                while (sub != null)
                {
                    if (sub.GetTypeName2() == "ProfileFeature")
                    {
                        Console.WriteLine("    sketch " + sub.Name);
                        var sk = (Sketch)sub.GetSpecificFeature2();
                        Console.WriteLine("      constrained status (3=fully): " + sk.GetConstrainedStatus());
                        var sdd = (DisplayDimension)sub.GetFirstDisplayDimension();
                        while (sdd != null)
                        {
                            var d = (Dimension)sdd.GetDimension2(0);
                            Console.WriteLine(string.Format("      {0} = {1:F4}", d.FullName, d.SystemValue * 1e3));
                            sdd = (DisplayDimension)sub.GetNextDisplayDimension(sdd);
                        }
                        object[] segs = (object[])sk.GetSketchSegments();
                        Console.WriteLine("      segments: " + (segs == null ? 0 : segs.Length));
                    }
                    sub = (Feature)sub.GetNextSubFeature();
                }
            }
            f = (Feature)f.GetNextFeature();
        }
    }
}
