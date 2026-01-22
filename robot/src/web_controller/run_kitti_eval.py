#!/usr/bin/env python3
import os
import subprocess
import sys
import shutil
from pathlib import Path

def prepare_kitti_structure(pred_dir, gt_dir, work_dir='kitti_workspace'):
    work_path = Path(work_dir)
    gt_label_dir = work_path / 'label_2'
    result_data_dir = work_path / 'results' / 'data'  # ← Ajout de /data
    
    if work_path.exists():
        shutil.rmtree(work_path)
    
    gt_label_dir.mkdir(parents=True, exist_ok=True)
    result_data_dir.mkdir(parents=True, exist_ok=True)
    
    print("📁 Préparation de la structure KITTI...")
    print(f"   GT dir: {gt_label_dir}")
    print(f"   Result dir: {result_data_dir}")
    
    gt_source = Path(gt_dir)
    gt_files = list(gt_source.glob('*.txt'))
    
    if not gt_files:
        print(f"❌ Aucun fichier .txt dans {gt_dir}")
        return None, None
    
    for f in gt_files:
        shutil.copy(f, gt_label_dir / f.name)
    
    print(f"  ✓ Ground truth: {len(gt_files)} fichiers")
    
    pred_source = Path(pred_dir)
    pred_files = list(pred_source.glob('*.txt'))
    
    if not pred_files:
        print(f"❌ Aucun fichier .txt dans {pred_dir}")
        return None, None
    
    for f in pred_files:
        shutil.copy(f, result_data_dir / f.name)  # ← Copie dans /data
    
    print(f"  ✓ Prédictions: {len(pred_files)} fichiers")
    
    gt_names = {f.name for f in gt_files}
    pred_names = {f.name for f in pred_files}
    common = gt_names & pred_names
    
    print(f"  ✓ Fichiers communs: {len(common)}")
    
    return work_path, work_path / 'results'

def run_evaluation(eval_bin, work_dir, result_dir):
    if not os.path.exists(eval_bin):
        print(f"❌ Exécutable introuvable: {eval_bin}")
        return False
    
    cmd = [str(eval_bin), str(work_dir), str(result_dir)]
    
    print("\n" + "=" * 80)
    print("🚀 ÉVALUATION KITTI 3D")
    print("=" * 80)
    print(f"Commande: {' '.join(cmd)}\n")
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, cwd=os.path.dirname(eval_bin))
        print(result.stdout)
        if result.stderr:
            print("Erreurs:", result.stderr)
        return result.returncode == 0
    except Exception as e:
        print(f"❌ Erreur: {e}")
        return False

def display_results(result_dir):
    plot_dir = Path(result_dir) / 'plot'
    
    if not plot_dir.exists():
        print(f"\n⚠️  Dossier 'plot' non trouvé dans {result_dir}")
        # Essayer de lister ce qui existe
        print(f"\nContenu de {result_dir}:")
        if Path(result_dir).exists():
            for item in Path(result_dir).iterdir():
                print(f"  - {item}")
        return
    
    print("\n" + "=" * 80)
    print("📊 RÉSULTATS")
    print("=" * 80)
    
    result_files = sorted(plot_dir.glob('*.txt'))
    
    if not result_files:
        print("Aucun fichier de résultat .txt trouvé")
        print(f"\nContenu de {plot_dir}:")
        for item in plot_dir.iterdir():
            print(f"  - {item}")
        return
    
    for result_file in result_files:
        print(f"\n{'='*40}")
        print(f"📄 {result_file.name}")
        print('='*40)
        with open(result_file, 'r') as f:
            content = f.read().strip()
            if content:
                print(content)
            else:
                print("(vide)")

def main():
    SCRIPT_DIR = Path(__file__).parent
    EVAL_BIN = SCRIPT_DIR / 'kitti_eval' / 'evaluate_object_3d_offline'
    PRED_DIR = SCRIPT_DIR / 'evaluation_data' / 'predictions'
    GT_DIR = SCRIPT_DIR / 'evaluation_data' / 'ground_truth'
    WORK_DIR = SCRIPT_DIR / 'kitti_workspace'
    
    print("=" * 80)
    print("ÉVALUATEUR KITTI 3D")
    print("=" * 80)
    
    if not EVAL_BIN.exists():
        print(f"❌ Exécutable introuvable: {EVAL_BIN}")
        return 1
    
    if not PRED_DIR.exists():
        print(f"❌ Dossier prédictions introuvable: {PRED_DIR}")
        return 1
    
    if not GT_DIR.exists():
        print(f"❌ Dossier ground truth introuvable: {GT_DIR}")
        return 1
    
    work_path, result_dir = prepare_kitti_structure(PRED_DIR, GT_DIR, WORK_DIR)
    
    if work_path is None:
        return 1
    
    success = run_evaluation(EVAL_BIN, work_path, result_dir)
    
    if success:
        display_results(result_dir)
        print("\n✅ Évaluation terminée!")
        print(f"📁 Workspace: {work_path}")
        print(f"📁 Résultats: {result_dir}/plot/")
        return 0
    else:
        print("\n❌ Échec")
        return 1

if __name__ == '__main__':
    sys.exit(main())
