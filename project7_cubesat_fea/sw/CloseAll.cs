// Closes every open SOLIDWORKS document WITHOUT saving (frees memory).
using System;
using SolidWorks.Interop.sldworks;
public static class CloseAll
{
    public static void Run(string outDir)
    {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        object[] docs = (object[])sw.GetDocuments();
        Console.WriteLine("open documents: " + (docs == null ? 0 : docs.Length));
        if (docs != null) foreach (ModelDoc2 d in docs) Console.WriteLine("  " + d.GetTitle() + (d.GetSaveFlag() ? "  (unsaved changes discarded)" : ""));
        sw.CloseAllDocuments(true);
    }
}
