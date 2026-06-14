(() => {
  if (!window.Vue || !Vue.createApp) {
    document.documentElement.classList.add('no-vue');
    return;
  }

  const { createApp } = Vue;

  const volumes = [
    {
      id: 'v1',
      no: '1',
      badge: '理论基础',
      label: '第一卷',
      title: '数学基础与控制理论基础',
      shortTitle: '第一卷',
      selectorText: '状态空间、稳定性与最优化',
      subtitle: '建立控制语言、状态空间与稳定性框架',
      chapterCount: '14 章',
      chapterSpan: '第 1 至 14 章',
      lead: '统一控制理论的基础概念、边界与方法。',
      bridge: '先把符号、系统描述和稳定性语言统一起来，再进入各类控制设计。',
      href: './volume1/index.html',
      accent: '#335c81',
      accentSoft: '#e7eef7',
      accentDeep: '#1e3d57',
      cover: '建立状态、输入、输出的统一语言。',
      summary: '理论底座。',
      highlights: [
        { label: '起点', text: '状态空间、输入输出' },
        { label: '主线', text: '稳定性、鲁棒性、最优性' },
        { label: '作用', text: '给后续各卷统一语言' },
      ],
    },
    {
      id: 'v2',
      no: '2',
      badge: '对象建模',
      label: '第二卷',
      title: '高超声速飞行器建模与飞行动力学',
      shortTitle: '第二卷',
      selectorText: '建模、运动学与推进耦合',
      subtitle: '从物理系统抽取控制对象接口',
      chapterCount: '12 章',
      chapterSpan: '第 15 至 25 章',
      lead: '拆解飞行器对象、环境与运动学。',
      bridge: '先理解飞行器本体、环境和运动学，再看控制器如何贴上去。',
      href: './volume2/index.html',
      accent: '#18786f',
      accentSoft: '#e3f5f2',
      accentDeep: '#0e4b45',
      cover: '从任务剖面到六自由度，逐层看对象。',
      summary: '对象层。',
      highlights: [
        { label: '起点', text: '任务、坐标、环境' },
        { label: '主线', text: '气动、推进与运动学' },
        { label: '作用', text: '整理成对象模型' },
      ],
    },
    {
      id: 'v3',
      no: '3',
      badge: '控制方法',
      label: '第三卷',
      title: '经典与现代飞行控制方法',
      shortTitle: '第三卷',
      selectorText: '经典、现代与非线性控制',
      subtitle: '把控制方法放进同一张地图',
      chapterCount: '14 章',
      chapterSpan: '第 26 至 39 章',
      lead: '按谱系、场景和边界梳理方法。',
      bridge: '先看方法谱系，再看适用前提和失效边界。',
      href: './volume3/index.html',
      accent: '#9a6d1c',
      accentSoft: '#f6ecd1',
      accentDeep: '#5d4010',
      cover: '先看架构，再看控制律和边界。',
      summary: '方法层。',
      highlights: [
        { label: '起点', text: '经典自动驾驶仪与频域分析' },
        { label: '主线', text: '鲁棒、调度、动态逆' },
        { label: '作用', text: '整理成方法地图' },
      ],
    },
    {
      id: 'v4',
      no: '4',
      badge: '约束安全',
      label: '第四卷',
      title: '约束、安全、容错与制导控制一体化',
      shortTitle: '第四卷',
      selectorText: '约束、安全、容错与制导',
      subtitle: '把可行性和安全性写进控制设计',
      chapterCount: '12 章',
      chapterSpan: '第 40 至 51 章',
      lead: '把约束、安全、容错和制导统一起来。',
      bridge: '先把约束和安全条件显式写出，再讨论性能和鲁棒性。',
      href: './volume4/index.html',
      accent: '#8b3f67',
      accentSoft: '#f5dfe9',
      accentDeep: '#522640',
      cover: '先约束，再安全，然后谈性能。',
      summary: '约束层。',
      highlights: [
        { label: '起点', text: '输入饱和与可行域' },
        { label: '主线', text: 'BLF、CBF、MPC、容错与分配' },
        { label: '作用', text: '把约束写进设计条件' },
      ],
    },
    {
      id: 'v5',
      no: '5',
      badge: '验证训练',
      label: '第五卷',
      title: '仿真验证、工程实现与研究训练',
      shortTitle: '第五卷',
      selectorText: '仿真、验证与研究训练',
      subtitle: '把结果变成证据链',
      chapterCount: '10 章',
      chapterSpan: '第 52 至 60 章',
      lead: '把仿真、验证、案例和论文阅读连起来。',
      bridge: '先建立仿真和验证标准，再把案例和复现放进统一流程。',
      href: './volume5/index.html',
      accent: '#4a5fb2',
      accentSoft: '#e4e8fb',
      accentDeep: '#2a3568',
      cover: '从仿真到复现，把结果说清楚。',
      summary: '验证层。',
      highlights: [
        { label: '起点', text: '仿真平台、指标与可信性' },
        { label: '主线', text: 'Monte Carlo、案例库、论文阅读' },
        { label: '作用', text: '整理成证据链' },
      ],
    },
  ];

  createApp({
    data() {
      return {
        activeVolume: null,
        volumes,
      };
    },
    created() {
      this.activeVolume = this.volumes[0];
    },
  }).mount('#app');
})();
