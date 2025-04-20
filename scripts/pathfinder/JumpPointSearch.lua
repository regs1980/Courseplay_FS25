---@class HybridAStar.JpsMotionPrimitives : HybridAStar.SimpleMotionPrimitives
HybridAStar.JpsMotionPrimitives = CpObject(HybridAStar.SimpleMotionPrimitives)
function HybridAStar.JpsMotionPrimitives:init(gridSize, deltaPosGoal, deltaThetaGoal, constraints, goal)
    -- similar to the A*, the possible motion primitives (the neighbors) are in all 8 directions.
    AStar.SimpleMotionPrimitives.init(self, gridSize)
    for i = 1, #self.primitives do
        -- if we moved to the current node using primitive[i], then the primitive we would need to go back to
        -- the parent is primitive[i].parent
        self.primitives[i].parent = self.primitives[i + 4] or self.primitives[i - 4]
        if self.primitives[i].dy == 0 then
            -- horizontal move
            self.primitives[i].neighbors = {
                -- unconditionally include natural neighbors.
                -- always need this neighbor to the left or right of the current node
                { check = nil, neighbor = { dx = self.primitives[i].dx, dy = 0 } },
                -- if there is an obstacle at 'check' if we got to the node through this primitive, then
                -- create a forced neighbors at 'forcedNeighbor'
                { check = { dx = 0, dy = self.gridSize }, neighbor = { dx = self.primitives[i].dx, dy = self.gridSize } },
                { check = { dx = 0, dy = -self.gridSize }, neighbor = { dx = self.primitives[i].dx, dy = -self.gridSize } }
            }
        elseif self.primitives[i].dx == 0 then
            -- vertical move
            self.primitives[i].neighbors = {
                -- unconditionally include natural neighbors.
                -- always need this neighbor to the left or right of the current node
                { check = nil, neighbor = { dx = 0, dy = self.primitives[i].dy } },
                -- forced neighbors are included only if the checked neighbors are invalid
                { check = { dx = self.gridSize, dy = 0 }, neighbor = { dx = self.gridSize, dy = self.primitives[i].dy } },
                { check = { dx = -self.gridSize, dy = 0 }, neighbor = { dx = -self.gridSize, dy = self.primitives[i].dy } }
            }
        else
            -- diagonal move
            self.primitives[i].neighbors = {
                -- unconditionally include natural neighbors.
                -- we always need this neighbor to the left or right of the current node, plus the one in the same direction
                { check = nil, neighbor = { dx = self.primitives[i].dx, dy = 0 } },
                { check = nil, neighbor = { dx = 0, dy = self.primitives[i].dy } },
                { check = nil, neighbor = { dx = self.primitives[i].dx, dy = self.primitives[i].dy } },
                -- forced neighbors are included only if the checked neighbors are invalid
                { check = { dx = -self.primitives[i].dx, dy = 0 },
                  neighbor = { dx = -self.primitives[i].dx, dy = self.primitives[i].dy } },
                { check = { dx = 0, dy = -self.primitives[i].dy },
                  neighbor = { dx = self.primitives[i].dx, dy = -self.primitives[i].dy } },
            }

        end
    end
    self.deltaPosGoal = deltaPosGoal
    self.deltaThetaGoal = deltaThetaGoal
    self.constraints = constraints
    self.goal = goal
end

function HybridAStar.JpsMotionPrimitives:getGridSize()
    return self.gridSize
end

function HybridAStar.JpsMotionPrimitives:isValidNode(x, y, t, constraints)
    local node = { x = x, y = y, t = t }
    return constraints:isValidNode(node, true, true)
end

-- Get the possible neighbors when coming from the predecessor node.
-- While the other HybridAStar derived algorithms use a real motion primitive, meaning it gives the relative
-- x, y and theta values which need to be added to the predecessor, JPS supplies the actual coordinates of
-- the successors here instead.
-- This is not the most elegant solution and the only reason we do this is to be able to reuse the whole
-- framework in HybridAStar.lua with JPS.
function HybridAStar.JpsMotionPrimitives:getPrimitives(node)
    local primitives = {}
    if node.pred then
        local x, y, t = node.x, node.y, node.t
        -- Node has a parent, we will prune some neighbours
        -- Gets the direction of move
        local dx = self.gridSize * (x - node.pred.x) / math.max(1, math.abs(x - node.pred.x))
        local dy = self.gridSize * (y - node.pred.y) / math.max(1, math.abs(y - node.pred.y))
        local dDiag = math.sqrt(dx * dx + dy * dy)
        local xOk, yOk = false, false
        if math.abs(dx) > 0.1 and math.abs(dy) > 0.1 then
            -- diagonal move
            if self:isValidNode(x, y + dy, t, self.constraints) then
                table.insert(primitives, { x = x, y = y + dy, t = math.atan2(dy, 0), d = math.abs(dy) })
                yOk = true
            end
            if self:isValidNode(x + dx, y, t, self.constraints) then
                table.insert(primitives, { x = x + dx, y = y, t = math.atan2(0, dx), d = math.abs(dx) })
                xOk = true
            end
            if xOk or yOk then
                table.insert(primitives, { x = x + dx, y = y + dy, t = math.atan2(dy, dx), d = dDiag })
            end
            -- Forced neighbors
            if not self:isValidNode(x - dx, y, t, self.constraints) and yOk then
                table.insert(primitives, { x = x - dx, y = y + dy, t = math.atan2(dy, -dx), d = dDiag })
                table.insert(JumpPointSearch.markers, { label = 'forced x - y +', x = x - dx, y = y + dy })
                node.forced = true
            end
            if not self:isValidNode(x, y - dy, t, self.constraints) and xOk then
                table.insert(primitives, { x = x + dx, y = y - dy, t = math.atan2(-dy, dx), d = dDiag })
                table.insert(JumpPointSearch.markers, { label = 'forced x + y -', x = x + dx, y = y - dy })
                node.forced = true
            end
        else
            if math.abs(dx) < 0.1 then
                -- move along the y axis
                if self:isValidNode(x, y + dy, t, self.constraints) then
                    table.insert(primitives, { x = x, y = y + dy, t = math.atan2(dy, 0), d = math.abs(dy) })
                end
                -- Forced neighbors
                dDiag = math.sqrt(dy * dy + self.gridSize * self.gridSize)
                if not self:isValidNode(x + self.gridSize, y, t, self.constraints) then
                    table.insert(primitives, { x = x + self.gridSize, y = y + dy,
                                               t = math.atan2(dy, self.gridSize), d = dDiag })
                    table.insert(JumpPointSearch.markers, { label = 'forced x +', x = x + self.gridSize, y = y + dy })
                    node.forced = true
                end
                if not self:isValidNode(x - self.gridSize, y, t, self.constraints) then
                    table.insert(primitives, { x = x - self.gridSize, y = y + dy,
                                               t = math.atan2(dy, -self.gridSize), d = dDiag })
                    table.insert(JumpPointSearch.markers, { label = 'forced x -', x = x - self.gridSize, y = y + dy })
                    node.forced = true
                end
            else
                -- move along the x axis
                if self:isValidNode(x + dx, y, t, self.constraints) then
                    table.insert(primitives, { x = x + dx, y = y, t = math.atan2(0, dx), d = math.abs(dx) })
                end
                -- Forced neighbors
                dDiag = math.sqrt(dx * dx + self.gridSize * self.gridSize)
                if not self:isValidNode(x, y + self.gridSize, t, self.constraints) then
                    table.insert(primitives, { x = x + dx, y = y + self.gridSize,
                                               t = math.atan2(self.gridSize, dx), d = dDiag })
                    table.insert(JumpPointSearch.markers, { label = 'forced y +', x = x + dx, y = y + self.gridSize })
                    node.forced = true
                end
                if not self:isValidNode(x, y - self.gridSize, t, self.constraints) then
                    table.insert(primitives, { x = x + dx, y = y - self.gridSize,
                                               t = math.atan2(-self.gridSize, dx), d = dDiag })
                    table.insert(JumpPointSearch.markers, { label = 'forced y -', x = x + dx, y = y - self.gridSize })
                    node.forced = true
                end
            end
        end
    else
        -- no parent, this is the start node
        for _, p in pairs(self.primitives) do
            -- JPS does not really use motion primitives, what we call primitives are actually the
            -- successors, with their real coordinates, not just a delta.
            table.insert(primitives, { x = node.x + p.dx, y = node.y + p.dy, t = p.dt, d = p.d })
        end
    end
    return primitives
end

function HybridAStar.JpsMotionPrimitives:jump(node, pred, recursionCounter)
    if recursionCounter and recursionCounter > 2 then
        return node, recursionCounter
    end
    recursionCounter = recursionCounter and recursionCounter + 1 or 1
    local x, y, t = node.x, node.y, node.t
    if not self:isValidNode(x, y, t, self.constraints) then
        return nil
    end
    if node:equals(self.goal, self.deltaPosGoal, self.deltaThetaGoal) then
        return node
    end
    local dx = x - pred.x
    local dy = y - pred.y
    if math.abs(dx) > 0.1 and math.abs(dy) > 0.1 then
        -- diagonal move
        if (self:isValidNode(x - dx, y + dy, t, self.constraints) and not self:isValidNode(x - dx, y, t, self.constraints)) or
                (self:isValidNode(x + dx, y - dy, t, self.constraints) and not self:isValidNode(x, y - dy, t, self.constraints)) then
            -- Current node is a jump point if one of its left or right neighbors ahead is forced
            return node
        end
    else
        if math.abs(dx) > 0.1 then
            -- move along the x axis
            if (self:isValidNode(x + dx, y + self.gridSize, t, self.constraints) and not self:isValidNode(x, y + self.gridSize, t, self.constraints)) or
                    (self:isValidNode(x + dx, y - self.gridSize, t, self.constraints) and not self:isValidNode(x, y - self.gridSize, t, self.constraints)) then
                -- Current node is a jump point if one of its left or right neighbors ahead is forced
                return node
            end
        else
            -- move along the y axis
            if (self:isValidNode(x + self.gridSize, y + dy, t, self.constraints) and not self:isValidNode(x + self.gridSize, y, t, self.constraints)) or
                    (self:isValidNode(x - self.gridSize, y + dy, t, self.constraints) and not self:isValidNode(x - self.gridSize, y, t, self.constraints)) then
                -- Current node is a jump point if one of its left or right neighbors ahead is forced
                return node
            end
        end
    end
    -- Recursive horizontal/vertical search
    if math.abs(dx) > 0.1 and math.abs(dy) > 0.1 then
        local nextNode = State3D.copy(node)
        nextNode.x = nextNode.x + dx
        nextNode.g = nextNode.g + dx
        if self:jump(nextNode, node, recursionCounter) then
            return node
        end
        nextNode = State3D.copy(node)
        nextNode.y = nextNode.y + dy
        nextNode.g = nextNode.g + dy
        if self:jump(nextNode, node, recursionCounter) then
            return node
        end
    end
    -- Recursive diagonal search
    if self:isValidNode(x + dx, y, t, self.constraints) or self:isValidNode(x, y + dy, t, self.constraints) then
        local nextNode = State3D.copy(node)
        nextNode.x = nextNode.x + dx
        nextNode.y = nextNode.y + dy

        nextNode.g = nextNode.g + dy
        return self:jump(nextNode, node, recursionCounter)
    end
end

function HybridAStar.JpsMotionPrimitives:createSuccessor(node, primitive, hitchLength)
    local neighbor = State3D(primitive.x, primitive.y, primitive.t)
    local jumpNode, jumps = self:jump(neighbor, node)
    primitive.d = jumps and jumps * primitive.d or primitive.d
    if jumpNode then
        return State3D(jumpNode.x, jumpNode.y, jumpNode.t, node.g, node, Gear.Forward, Steer.Straight,
                node:getNextTrailerHeading(self.gridSize, hitchLength))
    end
end
--- A Jump Point Search
---@class JumpPointSearch : AStar
JumpPointSearch = CpObject(AStar)
JumpPointSearch.markers = {}

function JumpPointSearch:init(vehicle, yieldAfter, maxIterations)
    AStar.init(self, vehicle, yieldAfter, maxIterations)
    JumpPointSearch.markers = {}
end

function JumpPointSearch:initRun(start, goal, turnRadius, allowReverse, constraints, hitchLength)
    self.motionPrimitives = HybridAStar.JpsMotionPrimitives(self.deltaPos, self.deltaPosGoal, self.deltaThetaGoal, constraints, goal)
    return AStar.initRun(self, start, goal, turnRadius, allowReverse, constraints, hitchLength)
end

function JumpPointSearch:getMotionPrimitives(turnRadius, allowReverse)
    return self.motionPrimitives
end

---@class HybridAStarWithJpsInTheMiddle : HybridAStarWithAStarInTheMiddle
HybridAStarWithJpsInTheMiddle = CpObject(HybridAStarWithAStarInTheMiddle)

function HybridAStarWithJpsInTheMiddle:init(hybridRange, yieldAfter, maxIterations, mustBeAccurate)
    HybridAStarWithAStarInTheMiddle.init(self, hybridRange, yieldAfter, maxIterations, mustBeAccurate)
end

function HybridAStarWithJpsInTheMiddle:getAStar()
    return JumpPointSearch(self.yieldAfter)
end
