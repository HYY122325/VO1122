function varargout = State(action, varargin)
% vo5L.State — 持久状态管理（替代原 computeVO 内部 persistent S5STATE）
%
% 用法：
%   st = vo5L.State('get', agentId);
%   vo5L.State('set', agentId, st);
%   vo5L.State('reset');

persistent S5STATE
if isempty(S5STATE)
    S5STATE = struct('ids',[],'data',{{}});
end

switch lower(action)
    case 'reset'
        S5STATE = struct('ids',[],'data',{{}});
        varargout = {[]};
        return;

    case 'get'
        if numel(varargin) < 1
            error('vo5L.State:get requires agentId');
        end
        id = varargin{1};
        st0 = default_state();

        k = find(S5STATE.ids == id, 1);
        if isempty(k)
            S5STATE.ids(end+1)  = id;
            S5STATE.data{end+1} = st0;
            st = st0;
        else
            st = S5STATE.data{k};
            st = ensure_fields(st, st0);
        end
        varargout = {st};
        return;

    case 'set'
        if numel(varargin) < 2
            error('vo5L.State:set requires agentId and st');
        end
        id = varargin{1};
        st = varargin{2};

        k = find(S5STATE.ids == id, 1);
        if isempty(k)
            S5STATE.ids(end+1)  = id;
            S5STATE.data{end+1} = st;
        else
            S5STATE.data{k} = st;
        end
        varargout = {[]};
        return;

    otherwise
        error('vo5L.State:UnknownAction', 'Unknown action: %s', action);
end

end

%% ==================== local helpers ====================
function st = default_state()
st = struct('prevObjHdg', NaN, 'prevStartHdg', NaN, 'prevObjScale', NaN, ...
            'overrides', [], ...
            'tsIds', [], ...
            'tsHeadings', [], 'tsHead0', [], ...
            'tsPhase', [], 'tsObsLeft', [], ...
            'tsRVo', zeros(0,2), 'tsRVopt', zeros(0,2), ...
            'colregs_mode', 0, ...
            'fsmRole', string([]), 'fsmScene', string([]), ...
            'srMem', struct('id',{},'locked',{},'scene',{},'role',{}, ...
                            'pendScene',{},'pendRole',{},'pendCount',{}, ...
                            'lastSeen',{},'lastCR',{}));
end

function st = ensure_fields(st, tmpl)
% 补齐历史版本可能缺失的字段（保持与原 get_agent_state 行为一致）
if ~isfield(st,'prevObjScale'), st.prevObjScale = tmpl.prevObjScale; end
if ~isfield(st,'srMem'),        st.srMem = tmpl.srMem; end
if ~isfield(st,'fsmRole'),      st.fsmRole = tmpl.fsmRole; end
if ~isfield(st,'fsmScene'),     st.fsmScene = tmpl.fsmScene; end

if ~isfield(st,'overrides'),    st.overrides = tmpl.overrides; end
if ~isfield(st,'tsIds'),        st.tsIds = tmpl.tsIds; end
if ~isfield(st,'tsHeadings'),   st.tsHeadings = tmpl.tsHeadings; end
if ~isfield(st,'tsHead0'),      st.tsHead0 = tmpl.tsHead0; end
if ~isfield(st,'tsPhase'),      st.tsPhase = tmpl.tsPhase; end
if ~isfield(st,'tsObsLeft'),    st.tsObsLeft = tmpl.tsObsLeft; end
if ~isfield(st,'tsRVo'),        st.tsRVo = tmpl.tsRVo; end
if ~isfield(st,'tsRVopt'),      st.tsRVopt = tmpl.tsRVopt; end
if ~isfield(st,'colregs_mode'), st.colregs_mode = tmpl.colregs_mode; end
end
