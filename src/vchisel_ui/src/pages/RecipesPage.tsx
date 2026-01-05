/**
 * 配方管理页面
 */

import { useState, useEffect } from 'react';
import { useAppStore } from '@/store/appStore';
import { rosBridge } from '@/services/rosBridge';
import type { Recipe } from '@/types';

export function RecipesPage() {
  const { recipes, setRecipes, currentRecipe, setCurrentRecipe } = useAppStore();
  const [loading, setLoading] = useState(false);
  const [message, setMessage] = useState('');
  const [editMode, setEditMode] = useState(false);
  const [editingRecipe, setEditingRecipe] = useState<Recipe | null>(null);

  useEffect(() => {
    loadRecipes();
  }, []);

  const loadRecipes = async () => {
    setLoading(true);
    try {
      const list = await rosBridge.listRecipes();
      setRecipes(list);
    } catch (error) {
      setMessage(`加载配方列表失败: ${error}`);
    } finally {
      setLoading(false);
    }
  };

  const handleLoadRecipe = async (name: string) => {
    setLoading(true);
    try {
      const recipe = await rosBridge.loadRecipe(name);
      setCurrentRecipe(recipe);
      setMessage(`已加载配方: ${name}`);
    } catch (error) {
      setMessage(`加载失败: ${error}`);
    } finally {
      setLoading(false);
    }
  };

  const handleEditRecipe = (recipe: Recipe) => {
    setEditingRecipe({ ...recipe });
    setEditMode(true);
  };

  const handleSaveRecipe = async () => {
    if (!editingRecipe) return;
    setLoading(true);
    try {
      await rosBridge.saveRecipe(editingRecipe);
      setMessage(`配方已保存: ${editingRecipe.name}`);
      setEditMode(false);
      setEditingRecipe(null);
      loadRecipes();
    } catch (error) {
      setMessage(`保存失败: ${error}`);
    } finally {
      setLoading(false);
    }
  };

  const handleDeleteRecipe = async (name: string) => {
    if (!confirm(`确定要删除配方 "${name}" 吗？`)) return;
    setLoading(true);
    try {
      await rosBridge.deleteRecipe(name);
      setMessage(`已删除配方: ${name}`);
      loadRecipes();
    } catch (error) {
      setMessage(`删除失败: ${error}`);
    } finally {
      setLoading(false);
    }
  };

  const handleCreateNew = () => {
    setEditingRecipe({
      name: 'new_recipe',
      version: '1.0.0',
      description: '',
      created_at: new Date().toISOString(),
      modified_at: new Date().toISOString(),
      camera_exposure: 10000,
      camera_gain: 1.0,
      detection_threshold: 0.5,
      min_area: 100,
      max_area: 10000,
      chisel_force: 50,
      chisel_angle: 45,
      chisel_depth: 5,
      approach_speed: 100,
      working_speed: 50,
      retreat_speed: 200,
    });
    setEditMode(true);
  };

  const updateField = (field: keyof Recipe, value: string | number) => {
    if (!editingRecipe) return;
    setEditingRecipe({ ...editingRecipe, [field]: value });
  };

  return (
    <div className="p-6 space-y-6">
      <div className="flex items-center justify-between">
        <h1 className="text-2xl font-bold">配方管理</h1>
        <button onClick={handleCreateNew} className="btn-primary">
          新建配方
        </button>
      </div>

      {/* 消息 */}
      {message && (
        <div className="card bg-bosch-gray-700">
          <p className="text-sm">{message}</p>
        </div>
      )}

      {/* 编辑模式 */}
      {editMode && editingRecipe && (
        <div className="card">
          <h3 className="text-lg font-medium mb-4">编辑配方</h3>

          <div className="grid grid-cols-2 gap-4">
            {/* 基本信息 */}
            <div className="space-y-3">
              <div>
                <label className="block text-sm text-bosch-gray-400 mb-1">名称</label>
                <input
                  type="text"
                  value={editingRecipe.name}
                  onChange={(e) => updateField('name', e.target.value)}
                  className="input-field w-full"
                />
              </div>
              <div>
                <label className="block text-sm text-bosch-gray-400 mb-1">描述</label>
                <input
                  type="text"
                  value={editingRecipe.description}
                  onChange={(e) => updateField('description', e.target.value)}
                  className="input-field w-full"
                />
              </div>
            </div>

            {/* 相机参数 */}
            <div className="space-y-3">
              <div>
                <label className="block text-sm text-bosch-gray-400 mb-1">曝光时间 (μs)</label>
                <input
                  type="number"
                  value={editingRecipe.camera_exposure}
                  onChange={(e) => updateField('camera_exposure', Number(e.target.value))}
                  className="input-field w-full"
                />
              </div>
              <div>
                <label className="block text-sm text-bosch-gray-400 mb-1">增益</label>
                <input
                  type="number"
                  step="0.1"
                  value={editingRecipe.camera_gain}
                  onChange={(e) => updateField('camera_gain', Number(e.target.value))}
                  className="input-field w-full"
                />
              </div>
            </div>

            {/* 检测参数 */}
            <div className="space-y-3">
              <div>
                <label className="block text-sm text-bosch-gray-400 mb-1">检测阈值</label>
                <input
                  type="number"
                  step="0.1"
                  value={editingRecipe.detection_threshold}
                  onChange={(e) => updateField('detection_threshold', Number(e.target.value))}
                  className="input-field w-full"
                />
              </div>
              <div>
                <label className="block text-sm text-bosch-gray-400 mb-1">最小面积</label>
                <input
                  type="number"
                  value={editingRecipe.min_area}
                  onChange={(e) => updateField('min_area', Number(e.target.value))}
                  className="input-field w-full"
                />
              </div>
            </div>

            {/* 凿击参数 */}
            <div className="space-y-3">
              <div>
                <label className="block text-sm text-bosch-gray-400 mb-1">凿击力 (N)</label>
                <input
                  type="number"
                  value={editingRecipe.chisel_force}
                  onChange={(e) => updateField('chisel_force', Number(e.target.value))}
                  className="input-field w-full"
                />
              </div>
              <div>
                <label className="block text-sm text-bosch-gray-400 mb-1">凿击角度 (°)</label>
                <input
                  type="number"
                  value={editingRecipe.chisel_angle}
                  onChange={(e) => updateField('chisel_angle', Number(e.target.value))}
                  className="input-field w-full"
                />
              </div>
            </div>
          </div>

          <div className="flex gap-4 mt-6">
            <button onClick={handleSaveRecipe} disabled={loading} className="btn-success">
              保存
            </button>
            <button onClick={() => setEditMode(false)} className="btn-secondary">
              取消
            </button>
          </div>
        </div>
      )}

      {/* 配方列表 */}
      {!editMode && (
        <div className="card">
          <h3 className="text-lg font-medium mb-4">配方列表</h3>
          {loading ? (
            <p className="text-bosch-gray-400">加载中...</p>
          ) : recipes.length === 0 ? (
            <p className="text-bosch-gray-400">暂无配方</p>
          ) : (
            <table className="w-full">
              <thead>
                <tr className="text-left text-bosch-gray-400 border-b border-bosch-gray-700">
                  <th className="pb-2">名称</th>
                  <th className="pb-2">版本</th>
                  <th className="pb-2">描述</th>
                  <th className="pb-2">修改时间</th>
                  <th className="pb-2">操作</th>
                </tr>
              </thead>
              <tbody>
                {recipes.map((recipe) => (
                  <tr key={recipe.name} className="border-b border-bosch-gray-700">
                    <td className="py-3">
                      <span className="font-medium">{recipe.name}</span>
                      {recipe.is_active && (
                        <span className="ml-2 px-2 py-0.5 bg-status-ok text-xs rounded">
                          当前
                        </span>
                      )}
                    </td>
                    <td className="py-3 text-bosch-gray-400">{recipe.version}</td>
                    <td className="py-3 text-bosch-gray-400">{recipe.description}</td>
                    <td className="py-3 text-bosch-gray-400 text-sm">{recipe.modified_at}</td>
                    <td className="py-3">
                      <div className="flex gap-2">
                        <button
                          onClick={() => handleLoadRecipe(recipe.name)}
                          className="text-sm text-status-info hover:underline"
                        >
                          加载
                        </button>
                        <button
                          onClick={async () => {
                            const r = await rosBridge.loadRecipe(recipe.name);
                            handleEditRecipe(r);
                          }}
                          className="text-sm text-status-warning hover:underline"
                        >
                          编辑
                        </button>
                        <button
                          onClick={() => handleDeleteRecipe(recipe.name)}
                          className="text-sm text-status-error hover:underline"
                        >
                          删除
                        </button>
                      </div>
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          )}
        </div>
      )}

      {/* 当前配方详情 */}
      {currentRecipe && !editMode && (
        <div className="card">
          <h3 className="text-lg font-medium mb-4">当前配方: {currentRecipe.name}</h3>
          <div className="grid grid-cols-3 gap-4 text-sm">
            <div>
              <span className="text-bosch-gray-400">曝光:</span>{' '}
              {currentRecipe.camera_exposure}μs
            </div>
            <div>
              <span className="text-bosch-gray-400">增益:</span>{' '}
              {currentRecipe.camera_gain}
            </div>
            <div>
              <span className="text-bosch-gray-400">凿击力:</span>{' '}
              {currentRecipe.chisel_force}N
            </div>
            <div>
              <span className="text-bosch-gray-400">凿击角度:</span>{' '}
              {currentRecipe.chisel_angle}°
            </div>
            <div>
              <span className="text-bosch-gray-400">凿击深度:</span>{' '}
              {currentRecipe.chisel_depth}mm
            </div>
            <div>
              <span className="text-bosch-gray-400">检测阈值:</span>{' '}
              {currentRecipe.detection_threshold}
            </div>
          </div>
        </div>
      )}
    </div>
  );
}
