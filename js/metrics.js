/**
 * js/metrics.js — Episode metrics tracking
 *
 * Logs per-episode statistics for training analysis and V2V comparison.
 */

export class EpisodeMetrics {
  constructor() {
    this.reset();
  }

  reset() {
    this.episode = 0;
    this.step = 0;
    this.totalReward = 0;
    this.collision = false;
    this.collisionType = '';
    this.minTTC = Infinity;
    this.minDistance = Infinity;
    this.avgSpeed = 0;
    this._speedSum = 0;
    this.progress = 0;
    this.maxProgress = 0;
    this.laneDeviation = 0;
    this._laneDevSum = 0;
    this.v2vMessagesReceived = 0;
    this.v2vOnlyDetections = 0;
    this.localDetections = 0;
    this.fusedDetections = 0;
    this.firstDetectionDistance = null;
    this.firstDetectionTime = null;
    this.firstHazardTime = null;
    this.TTCAtDetection = null;
    this.warningLeadTime = null;
    this.brakeStartTime = null;
    this.nearMiss = false;
    this.nearMissCount = 0;
    this.offRoadTime = 0;
    this.stuckTime = 0;
    this.startTime = 0;
    this.endTime = 0;
    // Scenario tags
    this.v2vEnabled = false;
    this.hasErratic = false;
    this.hasOccluded = false;
    this.curriculumStage = 1;
  }

  recordStep(stepData) {
    this.step++;
    this.totalReward += stepData.reward ?? 0;

    // Speed
    const speed = Math.abs(stepData.speed ?? 0);
    this._speedSum += speed;
    this.avgSpeed = this._speedSum / this.step;

    // Lane deviation
    this._laneDevSum += Math.abs(stepData.laneDev ?? 0);
    this.laneDeviation = this._laneDevSum / this.step;

    // Progress
    if (stepData.progress !== undefined) {
      this.progress = stepData.progress;
      this.maxProgress = Math.max(this.maxProgress, this.progress);
    }

    // TTC tracking
    if (stepData.minTTC !== undefined && stepData.minTTC < this.minTTC) {
      this.minTTC = stepData.minTTC;
    }

    // Distance tracking
    if (stepData.minDistance !== undefined && stepData.minDistance < this.minDistance) {
      this.minDistance = stepData.minDistance;
      if (this.minDistance < 3.0) {
        this.nearMiss = true;
        this.nearMissCount++;
      }
    }

    // Perception tracking
    if (stepData.v2vReceived) this.v2vMessagesReceived += stepData.v2vReceived;
    if (stepData.detectionCounts) {
      this.v2vOnlyDetections += stepData.detectionCounts.v2v ?? 0;
      this.localDetections += stepData.detectionCounts.local ?? 0;
      this.fusedDetections += stepData.detectionCounts.fused ?? 0;
    }

    // First detection tracking
    if (stepData.newDetection && this.firstDetectionTime === null) {
      this.firstDetectionTime = stepData.time;
      this.firstDetectionDistance = stepData.detectionDistance;
      this.TTCAtDetection = stepData.ttcAtDetection;
    }

    // Hazard timing
    if (stepData.hazardDetected && this.firstHazardTime === null) {
      this.firstHazardTime = stepData.time;
      if (this.firstDetectionTime !== null) {
        this.warningLeadTime = this.firstHazardTime - this.firstDetectionTime;
      }
    }

    // Brake timing
    if (stepData.braking && this.brakeStartTime === null) {
      this.brakeStartTime = stepData.time;
    }

    // Off-road tracking
    if (stepData.offRoad) {
      this.offRoadTime += stepData.dt ?? (1 / 60);
    }

    // Stuck tracking
    if (speed < 0.5) {
      this.stuckTime += stepData.dt ?? (1 / 60);
    } else {
      this.stuckTime = 0;
    }
  }

  recordCollision(type) {
    this.collision = true;
    this.collisionType = type;
  }

  finalize(time) {
    this.endTime = time;
  }

  /** Get a serializable summary */
  toJSON() {
    return {
      episode: this.episode,
      steps: this.step,
      reward: this.totalReward,
      collision: this.collision,
      collisionType: this.collisionType,
      minTTC: this.minTTC === Infinity ? null : this.minTTC,
      minDistance: this.minDistance === Infinity ? null : this.minDistance,
      avgSpeed: this.avgSpeed,
      progress: this.progress,
      laneDeviation: this.laneDeviation,
      v2vMessagesReceived: this.v2vMessagesReceived,
      v2vOnlyDetections: this.v2vOnlyDetections,
      localDetections: this.localDetections,
      fusedDetections: this.fusedDetections,
      firstDetectionDistance: this.firstDetectionDistance,
      firstDetectionTime: this.firstDetectionTime,
      firstHazardTime: this.firstHazardTime,
      TTCAtDetection: this.TTCAtDetection,
      warningLeadTime: this.warningLeadTime,
      brakeStartTime: this.brakeStartTime,
      nearMiss: this.nearMiss,
      nearMissCount: this.nearMissCount,
      duration: this.endTime - this.startTime,
      v2vEnabled: this.v2vEnabled,
      hasErratic: this.hasErratic,
      hasOccluded: this.hasOccluded,
      curriculumStage: this.curriculumStage
    };
  }
}

/**
 * Aggregated statistics across episodes.
 * Maintains separate buckets for V2V/non-V2V, normal/erratic, occluded.
 */
export class TrainingMetrics {
  constructor() {
    this.episodes = [];
    this.buckets = {
      v2v: [],
      noV2v: [],
      normal: [],
      erratic: [],
      occluded: [],
      all: []
    };
  }

  addEpisode(metrics) {
    const json = metrics.toJSON();
    this.episodes.push(json);
    this.buckets.all.push(json);

    if (json.v2vEnabled) this.buckets.v2v.push(json);
    else this.buckets.noV2v.push(json);

    if (json.hasErratic) this.buckets.erratic.push(json);
    else this.buckets.normal.push(json);

    if (json.hasOccluded) this.buckets.occluded.push(json);
  }

  /** Get summary statistics for a bucket */
  bucketStats(bucketName) {
    const bucket = this.buckets[bucketName] || [];
    if (bucket.length === 0) return null;

    const avg = (arr, key) => {
      const vals = arr.map(e => e[key]).filter(v => v !== null && v !== undefined);
      return vals.length ? vals.reduce((a, b) => a + b, 0) / vals.length : null;
    };

    return {
      count: bucket.length,
      avgReward: avg(bucket, 'reward'),
      collisionRate: bucket.filter(e => e.collision).length / bucket.length,
      avgMinTTC: avg(bucket, 'minTTC'),
      avgMinDistance: avg(bucket, 'minDistance'),
      avgSpeed: avg(bucket, 'avgSpeed'),
      avgProgress: avg(bucket, 'progress'),
      avgLaneDev: avg(bucket, 'laneDeviation'),
      nearMissRate: bucket.filter(e => e.nearMiss).length / bucket.length,
      avgFirstDetDist: avg(bucket, 'firstDetectionDistance'),
      avgWarningLead: avg(bucket, 'warningLeadTime')
    };
  }

  /** Get comparison between V2V and non-V2V */
  getV2VComparison() {
    return {
      v2v: this.bucketStats('v2v'),
      noV2v: this.bucketStats('noV2v'),
      normal: this.bucketStats('normal'),
      erratic: this.bucketStats('erratic'),
      occluded: this.bucketStats('occluded')
    };
  }

  /** Get the last N episodes */
  recent(n = 100) {
    return this.episodes.slice(-n);
  }

  /** Export all data as JSON */
  exportJSON() {
    return JSON.stringify({
      episodes: this.episodes,
      comparison: this.getV2VComparison()
    }, null, 2);
  }
}
