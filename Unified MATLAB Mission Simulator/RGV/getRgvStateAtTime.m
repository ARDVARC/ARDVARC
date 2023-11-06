function [pos, euler, movementType] = getRgvStateAtTime(this, time)
    prevStateIndex = find(this.times <= time, 1, "last");
    startTime = this.times(prevStateIndex);
    startPos = this.positions(prevStateIndex, :);
    startEul = this.eulers(prevStateIndex, :);
    movementType = this.movementTypes(prevStateIndex);

    [pos, euler] = moveRgv(time, startTime, startPos, startEul, movementType, this.params);
end