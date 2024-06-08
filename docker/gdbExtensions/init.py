






<!DOCTYPE html>
<html class="gl-light ui-neutral with-header with-top-bar " lang="en">
<head prefix="og: http://ogp.me/ns#">
<meta charset="utf-8">
<meta content="IE=edge" http-equiv="X-UA-Compatible">
<meta content="width=device-width, initial-scale=1" name="viewport">
<title>Files · master · libeigen / eigen · GitLab</title>
<script nonce="P8NEvu5zQj6vpUmdeGqmoA==">
//<![CDATA[
window.gon={};gon.math_rendering_limits_enabled=true;gon.features={"explainCodeChat":false};gon.licensed_features={"fileLocks":true,"remoteDevelopment":true};
//]]>
</script>


<script nonce="P8NEvu5zQj6vpUmdeGqmoA==">
//<![CDATA[
var gl = window.gl || {};
gl.startup_calls = {"/libeigen/eigen/-/refs/master/logs_tree/?format=json\u0026offset=0":{},"/libeigen/eigen/-/blob/master/README.md?format=json\u0026viewer=rich":{}};
gl.startup_graphql_calls = [{"query":"query pathLastCommit($projectPath: ID!, $path: String, $ref: String!, $refType: RefType) {\n  project(fullPath: $projectPath) {\n    __typename\n    id\n    repository {\n      __typename\n      paginatedTree(path: $path, ref: $ref, refType: $refType) {\n        __typename\n        nodes {\n          __typename\n          lastCommit {\n            __typename\n            id\n            sha\n            title\n            titleHtml\n            descriptionHtml\n            message\n            webPath\n            authoredDate\n            authorName\n            authorGravatar\n            author {\n              __typename\n              id\n              name\n              avatarUrl\n              webPath\n            }\n            signature {\n              __typename\n              ... on GpgSignature {\n                gpgKeyPrimaryKeyid\n                verificationStatus\n              }\n              ... on X509Signature {\n                verificationStatus\n                x509Certificate {\n                  id\n                  subject\n                  subjectKeyIdentifier\n                  x509Issuer {\n                    id\n                    subject\n                    subjectKeyIdentifier\n                  }\n                }\n              }\n              ... on SshSignature {\n                verificationStatus\n                keyFingerprintSha256\n              }\n            }\n            pipelines(ref: $ref, first: 1) {\n              __typename\n              edges {\n                __typename\n                node {\n                  __typename\n                  id\n                  detailedStatus {\n                    __typename\n                    id\n                    detailsPath\n                    icon\n                    tooltip\n                    text\n                    group\n                  }\n                }\n              }\n            }\n          }\n        }\n      }\n    }\n  }\n}\n","variables":{"projectPath":"libeigen/eigen","ref":"master","path":"","refType":null}},{"query":"query getPermissions($projectPath: ID!) {\n  project(fullPath: $projectPath) {\n    id\n    __typename\n    userPermissions {\n      __typename\n      pushCode\n      forkProject\n      createMergeRequestIn\n    }\n  }\n}\n","variables":{"projectPath":"libeigen/eigen"}},{"query":"fragment PageInfo on PageInfo {\n  __typename\n  hasNextPage\n  hasPreviousPage\n  startCursor\n  endCursor\n}\n\nfragment TreeEntry on Entry {\n  __typename\n  id\n  sha\n  name\n  flatPath\n  type\n}\n\nquery getFiles(\n  $projectPath: ID!\n  $path: String\n  $ref: String!\n  $refType: RefType\n  $pageSize: Int!\n  $nextPageCursor: String\n) {\n  project(fullPath: $projectPath) {\n    id\n    __typename\n    repository {\n      __typename\n      tree(path: $path, ref: $ref, refType: $refType) {\n        __typename\n        trees(first: $pageSize, after: $nextPageCursor) {\n          __typename\n          edges {\n            __typename\n            node {\n              ...TreeEntry\n              webPath\n            }\n          }\n          pageInfo {\n            ...PageInfo\n          }\n        }\n        submodules(first: $pageSize, after: $nextPageCursor) {\n          __typename\n          edges {\n            __typename\n            node {\n              ...TreeEntry\n              webUrl\n              treeUrl\n            }\n          }\n          pageInfo {\n            ...PageInfo\n          }\n        }\n        blobs(first: $pageSize, after: $nextPageCursor) {\n          __typename\n          edges {\n            __typename\n            node {\n              ...TreeEntry\n              mode\n              webPath\n              lfsOid\n            }\n          }\n          pageInfo {\n            ...PageInfo\n          }\n        }\n      }\n    }\n  }\n}\n","variables":{"nextPageCursor":"","pageSize":100,"projectPath":"libeigen/eigen","ref":"master","path":"/","refType":null}}];

if (gl.startup_calls && window.fetch) {
  Object.keys(gl.startup_calls).forEach(apiCall => {
   gl.startup_calls[apiCall] = {
      fetchCall: fetch(apiCall, {
        // Emulate XHR for Rails AJAX request checks
        headers: {
          'X-Requested-With': 'XMLHttpRequest'
        },
        // fetch won’t send cookies in older browsers, unless you set the credentials init option.
        // We set to `same-origin` which is default value in modern browsers.
        // See https://github.com/whatwg/fetch/pull/585 for more information.
        credentials: 'same-origin'
      })
    };
  });
}
if (gl.startup_graphql_calls && window.fetch) {
  const headers = {"X-CSRF-Token":"D29uVg9vndZ67xcMVKgiF8AXLSV3TGPsbnjHvatWK7HBkZxr2QnP4Vtc9h-rNW9ziLlfRAX-yxLP9wR6KJpLlw","x-gitlab-feature-category":"source_code_management"};
  const url = `https://gitlab.com/api/graphql`

  const opts = {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
      ...headers,
    }
  };

  gl.startup_graphql_calls = gl.startup_graphql_calls.map(call => ({
    ...call,
    fetchCall: fetch(url, {
      ...opts,
      credentials: 'same-origin',
      body: JSON.stringify(call)
    })
  }))
}


//]]>
</script>



<link rel="stylesheet" href="/assets/application-a364ce365b0117f79dcfa44667a7886a514a424db2b63606027984ac1c9ed8c6.css" media="all" />
<link rel="stylesheet" href="/assets/page_bundles/tree-4d25647d03722854e14fe89644330ef783d9e6e79f75ae79c5755c11825ddfc8.css" media="all" /><link rel="stylesheet" href="/assets/page_bundles/commit_description-5653213c51a6c90453a926cfc5e5e71ad9b41881a20a408bef8a303cf175435c.css" media="all" /><link rel="stylesheet" href="/assets/page_bundles/projects-891efed427b1a43cd34b53743b2cfbe37354acae2b0781df1516508f9096c0db.css" media="all" /><link rel="stylesheet" href="/assets/page_bundles/work_items-3fb37e34a79ad1c228c372bee00e2bd5402c17fd9e76678f1f29e0d6c82fc239.css" media="all" />
<link rel="stylesheet" href="/assets/application_utilities-d9a7b82eeacf5bfb8ceea81780b391ff56bc5b4332509ae17839f3590bce0ae9.css" media="all" />
<link rel="stylesheet" href="/assets/tailwind-7f75822cc78ba6631f79321275dc6092ef4f1fe0ac173656389342b8901f496f.css" media="all" />


<link rel="stylesheet" href="/assets/fonts-fae5d3f79948bd85f18b6513a025f863b19636e85b09a1492907eb4b1bb0557b.css" media="all" />
<link rel="stylesheet" href="/assets/highlight/themes/white-e08c45a78f4446ec6c4226adb581d4482911bd7c85b47b7e7c003112b0c26274.css" media="all" />

<script src="/assets/webpack/runtime.b5dbb1d8.bundle.js" defer="defer" nonce="P8NEvu5zQj6vpUmdeGqmoA=="></script>
<script src="/assets/webpack/main.81b0b354.chunk.js" defer="defer" nonce="P8NEvu5zQj6vpUmdeGqmoA=="></script>
<script src="/assets/webpack/tracker.0204876f.chunk.js" defer="defer" nonce="P8NEvu5zQj6vpUmdeGqmoA=="></script>
<script src="/assets/webpack/analytics.1d646133.chunk.js" defer="defer" nonce="P8NEvu5zQj6vpUmdeGqmoA=="></script>
<script nonce="P8NEvu5zQj6vpUmdeGqmoA==">
//<![CDATA[
window.snowplowOptions = {"namespace":"gl","hostname":"snowplow.trx.gitlab.net","cookieDomain":".gitlab.com","appId":"gitlab","formTracking":true,"linkClickTracking":true}

gl = window.gl || {};
gl.snowplowStandardContext = {"schema":"iglu:com.gitlab/gitlab_standard/jsonschema/1-0-10","data":{"environment":"production","source":"gitlab-rails","plan":"free","extra":{},"user_id":null,"is_gitlab_team_member":null,"namespace_id":6577938,"project_id":15462818,"feature_enabled_by_namespace_ids":null,"context_generated_at":"2024-06-06T22:43:54.477Z"}}
gl.snowplowPseudonymizedPageUrl = "https://gitlab.com/namespace6577938/project15462818/-/tree/master";
gl.maskedDefaultReferrerUrl = null;


//]]>
</script>
<link rel="preload" href="/assets/application_utilities-d9a7b82eeacf5bfb8ceea81780b391ff56bc5b4332509ae17839f3590bce0ae9.css" as="style" type="text/css" nonce="dBUMhK7QOdD3y4VkQghuhQ==">
<link rel="preload" href="/assets/application-a364ce365b0117f79dcfa44667a7886a514a424db2b63606027984ac1c9ed8c6.css" as="style" type="text/css" nonce="dBUMhK7QOdD3y4VkQghuhQ==">
<link rel="preload" href="/assets/highlight/themes/white-e08c45a78f4446ec6c4226adb581d4482911bd7c85b47b7e7c003112b0c26274.css" as="style" type="text/css" nonce="dBUMhK7QOdD3y4VkQghuhQ==">
<link crossorigin="" href="https://snowplow.trx.gitlab.net" rel="preconnect">
<link as="font" crossorigin="" href="/assets/gitlab-sans/GitLabSans-1e0a5107ea3bbd4be93e8ad2c503467e43166cd37e4293570b490e0812ede98b.woff2" rel="preload">
<link as="font" crossorigin="" href="/assets/gitlab-sans/GitLabSans-Italic-38eaf1a569a54ab28c58b92a4a8de3afb96b6ebc250cf372003a7b38151848cc.woff2" rel="preload">
<link as="font" crossorigin="" href="/assets/gitlab-mono/GitLabMono-08d2c5e8ff8fd3d2d6ec55bc7713380f8981c35f9d2df14e12b835464d6e8f23.woff2" rel="preload">
<link as="font" crossorigin="" href="/assets/gitlab-mono/GitLabMono-Italic-38e58d8df29485a20c550da1d0111e2c2169f6dcbcf894f2cd3afbdd97bcc588.woff2" rel="preload">
<link rel="preload" href="/assets/fonts-fae5d3f79948bd85f18b6513a025f863b19636e85b09a1492907eb4b1bb0557b.css" as="style" type="text/css" nonce="dBUMhK7QOdD3y4VkQghuhQ==">



<script src="/assets/webpack/sentry.dc11e1fd.chunk.js" defer="defer" nonce="P8NEvu5zQj6vpUmdeGqmoA=="></script>


<script src="/assets/webpack/commons-pages.admin.abuse_reports.show-pages.dashboard.issues-pages.dashboard.milestones.show-pages.-6eb574d2.b3233192.chunk.js" defer="defer" nonce="P8NEvu5zQj6vpUmdeGqmoA=="></script>
<script src="/assets/webpack/commons-pages.admin.abuse_reports.show-pages.dashboard.issues-pages.groups.boards-pages.groups.epic_-702584b3.811e275d.chunk.js" defer="defer" nonce="P8NEvu5zQj6vpUmdeGqmoA=="></script>
<script src="/assets/webpack/commons-pages.admin.abuse_reports.show-pages.dashboard.issues-pages.groups.boards-pages.groups.epic_-67fb21ec.4b6757e2.chunk.js" defer="defer" nonce="P8NEvu5zQj6vpUmdeGqmoA=="></script>
<script src="/assets/webpack/commons-pages.groups.analytics.dashboards-pages.groups.analytics.dashboards.value_streams_dashboard--844b062c.1ec2e1f3.chunk.js" defer="defer" nonce="P8NEvu5zQj6vpUmdeGqmoA=="></script>
<script src="/assets/webpack/commons-pages.groups.new-pages.import.gitlab_projects.new-pages.import.manifest.new-pages.projects.n-44c6c18e.4dbeeabf.chunk.js" defer="defer" nonce="P8NEvu5zQj6vpUmdeGqmoA=="></script>
<script src="/assets/webpack/commons-pages.search.show-super_sidebar.b16e1339.chunk.js" defer="defer" nonce="P8NEvu5zQj6vpUmdeGqmoA=="></script>
<script src="/assets/webpack/super_sidebar.288162b2.chunk.js" defer="defer" nonce="P8NEvu5zQj6vpUmdeGqmoA=="></script>
<script src="/assets/webpack/commons-pages.projects-pages.projects.activity-pages.projects.alert_management.details-pages.project-6c5bf499.c727a54e.chunk.js" defer="defer" nonce="P8NEvu5zQj6vpUmdeGqmoA=="></script>
<script src="/assets/webpack/commons-pages.admin.runners.show-pages.clusters.agents.dashboard-pages.explore.catalog-pages.explore-b0499256.65d4090f.chunk.js" defer="defer" nonce="P8NEvu5zQj6vpUmdeGqmoA=="></script>
<script src="/assets/webpack/commons-pages.groups.security.policies.edit-pages.groups.security.policies.new-pages.projects.blob.s-c8e0a3ae.a4e25915.chunk.js" defer="defer" nonce="P8NEvu5zQj6vpUmdeGqmoA=="></script>
<script src="/assets/webpack/117.a8a6b28a.chunk.js" defer="defer" nonce="P8NEvu5zQj6vpUmdeGqmoA=="></script>
<script src="/assets/webpack/commons-pages.projects.blob.show-pages.projects.show-pages.projects.snippets.show-pages.projects.tre-c684fcf6.95bf4417.chunk.js" defer="defer" nonce="P8NEvu5zQj6vpUmdeGqmoA=="></script>
<script src="/assets/webpack/139.84a85749.chunk.js" defer="defer" nonce="P8NEvu5zQj6vpUmdeGqmoA=="></script>
<script src="/assets/webpack/commons-pages.projects.blob.show-pages.projects.show-pages.projects.tree.show.1e48748d.chunk.js" defer="defer" nonce="P8NEvu5zQj6vpUmdeGqmoA=="></script>
<script src="/assets/webpack/treeList.3427a7a9.chunk.js" defer="defer" nonce="P8NEvu5zQj6vpUmdeGqmoA=="></script>
<script src="/assets/webpack/commons-pages.projects.show-pages.projects.tree.show.1711b83a.chunk.js" defer="defer" nonce="P8NEvu5zQj6vpUmdeGqmoA=="></script>
<script src="/assets/webpack/pages.projects.tree.show.c66efa99.chunk.js" defer="defer" nonce="P8NEvu5zQj6vpUmdeGqmoA=="></script>
<meta content="object" property="og:type">
<meta content="GitLab" property="og:site_name">
<meta content="Files · master · libeigen / eigen · GitLab" property="og:title">
<meta content="Eigen is a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms." property="og:description">
<meta content="https://gitlab.com/uploads/-/system/project/avatar/15462818/Eigen_Silly_Professor_64x64.png" property="og:image">
<meta content="64" property="og:image:width">
<meta content="64" property="og:image:height">
<meta content="https://gitlab.com/libeigen/eigen/-/tree/master" property="og:url">
<meta content="summary" property="twitter:card">
<meta content="Files · master · libeigen / eigen · GitLab" property="twitter:title">
<meta content="Eigen is a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms." property="twitter:description">
<meta content="https://gitlab.com/uploads/-/system/project/avatar/15462818/Eigen_Silly_Professor_64x64.png" property="twitter:image">

<meta name="csrf-param" content="authenticity_token" />
<meta name="csrf-token" content="8_q7mhfizaUQvAPUP2AJURdBUYBw-z0u5ndkfQIhf5Y9BEmnwYSfkjEP4sfA_UQ1X-8j4QJJldBH-Ke6ge0fsA" />
<meta name="csp-nonce" content="P8NEvu5zQj6vpUmdeGqmoA==" />
<meta name="action-cable-url" content="/-/cable" />
<link href="/-/manifest.json" rel="manifest">
<link rel="icon" type="image/png" href="/assets/favicon-yellow-018213ceb87b472388095d0264be5b4319ef47471dacea03c83ecc233ced2fd5.png" id="favicon" data-original-href="/assets/favicon-yellow-018213ceb87b472388095d0264be5b4319ef47471dacea03c83ecc233ced2fd5.png" />
<link rel="apple-touch-icon" type="image/x-icon" href="/assets/apple-touch-icon-b049d4bc0dd9626f31db825d61880737befc7835982586d015bded10b4435460.png" />
<link href="/search/opensearch.xml" rel="search" title="Search GitLab" type="application/opensearchdescription+xml">
<link rel="alternate" type="application/atom+xml" title="eigen:master commits" href="https://gitlab.com/libeigen/eigen/-/commits/master?format=atom" />




<meta content="Eigen is a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms." name="description">
<meta content="#ececef" name="theme-color">
</head>

<body class="tab-width-8 gl-browser-generic gl-platform-other " data-find-file="/libeigen/eigen/-/find_file/master" data-group="libeigen" data-group-full-path="libeigen" data-namespace-id="6577938" data-page="projects:tree:show" data-page-type-id="master" data-project="eigen" data-project-full-path="libeigen/eigen" data-project-id="15462818">

<script nonce="P8NEvu5zQj6vpUmdeGqmoA==">
//<![CDATA[
gl = window.gl || {};
gl.client = {"isGeneric":true,"isOther":true};


//]]>
</script>


<header class="header-logged-out" data-testid="navbar">
<a class="gl-sr-only gl-accessibility" href="#content-body">Skip to content</a>
<div class="container-fluid">
<nav aria-label="Explore GitLab" class="header-logged-out-nav gl-display-flex gl-gap-3 gl-justify-content-space-between">
<div class="gl-display-flex gl-align-items-center gl-gap-1">
<span class="gl-sr-only">GitLab</span>
<a title="Homepage" id="logo" class="header-logged-out-logo has-tooltip" aria-label="Homepage" data-track-label="main_navigation" data-track-action="click_gitlab_logo_link" data-track-property="navigation_top" href="/"><svg aria-hidden="true" role="img" class="tanuki-logo" width="25" height="24" viewBox="0 0 25 24" fill="none" xmlns="http://www.w3.org/2000/svg">
  <path class="tanuki-shape tanuki" d="m24.507 9.5-.034-.09L21.082.562a.896.896 0 0 0-1.694.091l-2.29 7.01H7.825L5.535.653a.898.898 0 0 0-1.694-.09L.451 9.411.416 9.5a6.297 6.297 0 0 0 2.09 7.278l.012.01.03.022 5.16 3.867 2.56 1.935 1.554 1.176a1.051 1.051 0 0 0 1.268 0l1.555-1.176 2.56-1.935 5.197-3.89.014-.01A6.297 6.297 0 0 0 24.507 9.5Z"
        fill="#E24329"/>
  <path class="tanuki-shape right-cheek" d="m24.507 9.5-.034-.09a11.44 11.44 0 0 0-4.56 2.051l-7.447 5.632 4.742 3.584 5.197-3.89.014-.01A6.297 6.297 0 0 0 24.507 9.5Z"
        fill="#FC6D26"/>
  <path class="tanuki-shape chin" d="m7.707 20.677 2.56 1.935 1.555 1.176a1.051 1.051 0 0 0 1.268 0l1.555-1.176 2.56-1.935-4.743-3.584-4.755 3.584Z"
        fill="#FCA326"/>
  <path class="tanuki-shape left-cheek" d="M5.01 11.461a11.43 11.43 0 0 0-4.56-2.05L.416 9.5a6.297 6.297 0 0 0 2.09 7.278l.012.01.03.022 5.16 3.867 4.745-3.584-7.444-5.632Z"
        fill="#FC6D26"/>
</svg>

</a><a class="gl-badge badge badge-pill badge-success sm canary-badge" data-testid="canary_badge_link" href="https://next.gitlab.com" rel="noopener noreferrer" target="_blank">Next
</a></div>
<ul class="gl-list-none gl-p-0 gl-m-0 gl-display-flex gl-gap-3 gl-align-items-center gl-flex-grow-1">
<li class="header-logged-out-nav-item header-logged-out-dropdown md:gl-hidden">
<button class="header-logged-out-toggle" data-toggle="dropdown" type="button">
<span class="gl-sr-only">
Menu
</span>
<svg class="s16" data-testid="hamburger-icon"><use href="/assets/icons-0b41337f52be73f7bbf9d59b841eb98a6e790dfa1a844644f120a80ce3cc18ba.svg#hamburger"></use></svg>
</button>
<div class="dropdown-menu">
<ul>
<li>
<a href="https://about.gitlab.com/why-gitlab">Why GitLab
</a></li>
<li>
<a href="https://about.gitlab.com/pricing">Pricing
</a></li>
<li>
<a href="https://about.gitlab.com/sales">Contact Sales
</a></li>
<li>
<a href="/explore">Explore</a>
</li>
</ul>
</div>
</li>
<li class="header-logged-out-nav-item gl-hidden md:gl-inline-block">
<a href="https://about.gitlab.com/why-gitlab">Why GitLab
</a></li>
<li class="header-logged-out-nav-item gl-hidden md:gl-inline-block">
<a href="https://about.gitlab.com/pricing">Pricing
</a></li>
<li class="header-logged-out-nav-item gl-hidden gl-inline-block">
<a href="https://about.gitlab.com/sales">Contact Sales
</a></li>
<li class="header-logged-out-nav-item gl-hidden md:gl-inline-block">
<a class="" href="/explore">Explore</a>
</li>
</ul>
<ul class="gl-list-none gl-p-0 gl-m-0 gl-display-flex gl-gap-3 gl-align-items-center gl-justify-content-end">
<li class="header-logged-out-nav-item">
<a href="/users/sign_in?redirect_to_referer=yes">Sign in</a>
</li>
<li class="header-logged-out-nav-item">
<a class="gl-button btn btn-md btn-confirm " href="/users/sign_up"><span class="gl-button-text">
Get free trial

</span>

</a></li>
</ul>
</nav>
</div>
</header>

<div class="layout-page page-with-super-sidebar">
<aside class="js-super-sidebar super-sidebar super-sidebar-loading" data-command-palette="{&quot;project_files_url&quot;:&quot;/libeigen/eigen/-/files/master?format=json&quot;,&quot;project_blob_url&quot;:&quot;/libeigen/eigen/-/blob/master&quot;}" data-force-desktop-expanded-sidebar="" data-root-path="/" data-sidebar="{&quot;is_logged_in&quot;:false,&quot;context_switcher_links&quot;:[{&quot;title&quot;:&quot;Explore&quot;,&quot;link&quot;:&quot;/explore&quot;,&quot;icon&quot;:&quot;compass&quot;}],&quot;current_menu_items&quot;:[{&quot;id&quot;:&quot;project_overview&quot;,&quot;title&quot;:&quot;eigen&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:&quot;/uploads/-/system/project/avatar/15462818/Eigen_Silly_Professor_64x64.png&quot;,&quot;entity_id&quot;:15462818,&quot;link&quot;:&quot;/libeigen/eigen&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:&quot;shortcuts-project&quot;,&quot;is_active&quot;:false},{&quot;id&quot;:&quot;manage_menu&quot;,&quot;title&quot;:&quot;Manage&quot;,&quot;icon&quot;:&quot;users&quot;,&quot;avatar&quot;:null,&quot;avatar_shape&quot;:&quot;rect&quot;,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/activity&quot;,&quot;is_active&quot;:false,&quot;pill_count&quot;:null,&quot;items&quot;:[{&quot;id&quot;:&quot;activity&quot;,&quot;title&quot;:&quot;Activity&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/activity&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:&quot;shortcuts-project-activity&quot;,&quot;is_active&quot;:false},{&quot;id&quot;:&quot;members&quot;,&quot;title&quot;:&quot;Members&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/project_members&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:null,&quot;is_active&quot;:false},{&quot;id&quot;:&quot;labels&quot;,&quot;title&quot;:&quot;Labels&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/labels&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:null,&quot;is_active&quot;:false}],&quot;separated&quot;:false},{&quot;id&quot;:&quot;plan_menu&quot;,&quot;title&quot;:&quot;Plan&quot;,&quot;icon&quot;:&quot;planning&quot;,&quot;avatar&quot;:null,&quot;avatar_shape&quot;:&quot;rect&quot;,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/issues&quot;,&quot;is_active&quot;:false,&quot;pill_count&quot;:null,&quot;items&quot;:[{&quot;id&quot;:&quot;project_issue_list&quot;,&quot;title&quot;:&quot;Issues&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/issues&quot;,&quot;pill_count&quot;:&quot;633&quot;,&quot;link_classes&quot;:&quot;shortcuts-issues has-sub-items&quot;,&quot;is_active&quot;:false},{&quot;id&quot;:&quot;boards&quot;,&quot;title&quot;:&quot;Issue boards&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/boards&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:&quot;shortcuts-issue-boards&quot;,&quot;is_active&quot;:false},{&quot;id&quot;:&quot;milestones&quot;,&quot;title&quot;:&quot;Milestones&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/milestones&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:null,&quot;is_active&quot;:false},{&quot;id&quot;:&quot;requirements&quot;,&quot;title&quot;:&quot;Requirements&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/requirements_management/requirements&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:null,&quot;is_active&quot;:false},{&quot;id&quot;:&quot;external_issue_tracker&quot;,&quot;title&quot;:&quot;Custom issue tracker&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;http://manao.inria.fr/eigen_tmp/pullrequests&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:&quot;shortcuts-external_tracker&quot;,&quot;is_active&quot;:false}],&quot;separated&quot;:false},{&quot;id&quot;:&quot;code_menu&quot;,&quot;title&quot;:&quot;Code&quot;,&quot;icon&quot;:&quot;code&quot;,&quot;avatar&quot;:null,&quot;avatar_shape&quot;:&quot;rect&quot;,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/merge_requests&quot;,&quot;is_active&quot;:true,&quot;pill_count&quot;:null,&quot;items&quot;:[{&quot;id&quot;:&quot;project_merge_request_list&quot;,&quot;title&quot;:&quot;Merge requests&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/merge_requests&quot;,&quot;pill_count&quot;:&quot;23&quot;,&quot;link_classes&quot;:&quot;shortcuts-merge_requests&quot;,&quot;is_active&quot;:false},{&quot;id&quot;:&quot;files&quot;,&quot;title&quot;:&quot;Repository&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/tree/master&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:&quot;shortcuts-tree&quot;,&quot;is_active&quot;:true},{&quot;id&quot;:&quot;branches&quot;,&quot;title&quot;:&quot;Branches&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/branches&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:null,&quot;is_active&quot;:false},{&quot;id&quot;:&quot;commits&quot;,&quot;title&quot;:&quot;Commits&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/commits/master?ref_type=heads&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:&quot;shortcuts-commits&quot;,&quot;is_active&quot;:false},{&quot;id&quot;:&quot;tags&quot;,&quot;title&quot;:&quot;Tags&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/tags&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:null,&quot;is_active&quot;:false},{&quot;id&quot;:&quot;graphs&quot;,&quot;title&quot;:&quot;Repository graph&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/network/master?ref_type=heads&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:&quot;shortcuts-network&quot;,&quot;is_active&quot;:false},{&quot;id&quot;:&quot;compare&quot;,&quot;title&quot;:&quot;Compare revisions&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/compare?from=master\u0026to=master&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:null,&quot;is_active&quot;:false},{&quot;id&quot;:&quot;project_snippets&quot;,&quot;title&quot;:&quot;Snippets&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/snippets&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:&quot;shortcuts-snippets&quot;,&quot;is_active&quot;:false},{&quot;id&quot;:&quot;file_locks&quot;,&quot;title&quot;:&quot;Locked files&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/path_locks&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:null,&quot;is_active&quot;:false}],&quot;separated&quot;:false},{&quot;id&quot;:&quot;build_menu&quot;,&quot;title&quot;:&quot;Build&quot;,&quot;icon&quot;:&quot;rocket&quot;,&quot;avatar&quot;:null,&quot;avatar_shape&quot;:&quot;rect&quot;,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/pipelines&quot;,&quot;is_active&quot;:false,&quot;pill_count&quot;:null,&quot;items&quot;:[{&quot;id&quot;:&quot;pipelines&quot;,&quot;title&quot;:&quot;Pipelines&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/pipelines&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:&quot;shortcuts-pipelines&quot;,&quot;is_active&quot;:false},{&quot;id&quot;:&quot;jobs&quot;,&quot;title&quot;:&quot;Jobs&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/jobs&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:&quot;shortcuts-builds&quot;,&quot;is_active&quot;:false},{&quot;id&quot;:&quot;pipeline_schedules&quot;,&quot;title&quot;:&quot;Pipeline schedules&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/pipeline_schedules&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:&quot;shortcuts-builds&quot;,&quot;is_active&quot;:false},{&quot;id&quot;:&quot;test_cases&quot;,&quot;title&quot;:&quot;Test cases&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/quality/test_cases&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:&quot;shortcuts-test-cases&quot;,&quot;is_active&quot;:false},{&quot;id&quot;:&quot;artifacts&quot;,&quot;title&quot;:&quot;Artifacts&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/artifacts&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:&quot;shortcuts-builds&quot;,&quot;is_active&quot;:false}],&quot;separated&quot;:false},{&quot;id&quot;:&quot;deploy_menu&quot;,&quot;title&quot;:&quot;Deploy&quot;,&quot;icon&quot;:&quot;deployments&quot;,&quot;avatar&quot;:null,&quot;avatar_shape&quot;:&quot;rect&quot;,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/releases&quot;,&quot;is_active&quot;:false,&quot;pill_count&quot;:null,&quot;items&quot;:[{&quot;id&quot;:&quot;releases&quot;,&quot;title&quot;:&quot;Releases&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/releases&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:&quot;shortcuts-deployments-releases&quot;,&quot;is_active&quot;:false},{&quot;id&quot;:&quot;packages_registry&quot;,&quot;title&quot;:&quot;Package Registry&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/packages&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:&quot;shortcuts-container-registry&quot;,&quot;is_active&quot;:false},{&quot;id&quot;:&quot;container_registry&quot;,&quot;title&quot;:&quot;Container Registry&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/container_registry&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:null,&quot;is_active&quot;:false},{&quot;id&quot;:&quot;model_registry&quot;,&quot;title&quot;:&quot;Model registry&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/ml/models&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:null,&quot;is_active&quot;:false}],&quot;separated&quot;:false},{&quot;id&quot;:&quot;operations_menu&quot;,&quot;title&quot;:&quot;Operate&quot;,&quot;icon&quot;:&quot;cloud-pod&quot;,&quot;avatar&quot;:null,&quot;avatar_shape&quot;:&quot;rect&quot;,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/environments&quot;,&quot;is_active&quot;:false,&quot;pill_count&quot;:null,&quot;items&quot;:[{&quot;id&quot;:&quot;environments&quot;,&quot;title&quot;:&quot;Environments&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/environments&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:&quot;shortcuts-environments&quot;,&quot;is_active&quot;:false},{&quot;id&quot;:&quot;infrastructure_registry&quot;,&quot;title&quot;:&quot;Terraform modules&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/terraform_module_registry&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:null,&quot;is_active&quot;:false}],&quot;separated&quot;:false},{&quot;id&quot;:&quot;monitor_menu&quot;,&quot;title&quot;:&quot;Monitor&quot;,&quot;icon&quot;:&quot;monitor&quot;,&quot;avatar&quot;:null,&quot;avatar_shape&quot;:&quot;rect&quot;,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/incidents&quot;,&quot;is_active&quot;:false,&quot;pill_count&quot;:null,&quot;items&quot;:[{&quot;id&quot;:&quot;incidents&quot;,&quot;title&quot;:&quot;Incidents&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/incidents&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:null,&quot;is_active&quot;:false},{&quot;id&quot;:&quot;service_desk&quot;,&quot;title&quot;:&quot;Service Desk&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/issues/service_desk&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:null,&quot;is_active&quot;:false}],&quot;separated&quot;:false},{&quot;id&quot;:&quot;analyze_menu&quot;,&quot;title&quot;:&quot;Analyze&quot;,&quot;icon&quot;:&quot;chart&quot;,&quot;avatar&quot;:null,&quot;avatar_shape&quot;:&quot;rect&quot;,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/value_stream_analytics&quot;,&quot;is_active&quot;:false,&quot;pill_count&quot;:null,&quot;items&quot;:[{&quot;id&quot;:&quot;cycle_analytics&quot;,&quot;title&quot;:&quot;Value stream analytics&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/value_stream_analytics&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:&quot;shortcuts-project-cycle-analytics&quot;,&quot;is_active&quot;:false},{&quot;id&quot;:&quot;contributors&quot;,&quot;title&quot;:&quot;Contributor analytics&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/graphs/master?ref_type=heads&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:null,&quot;is_active&quot;:false},{&quot;id&quot;:&quot;ci_cd_analytics&quot;,&quot;title&quot;:&quot;CI/CD analytics&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/pipelines/charts&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:null,&quot;is_active&quot;:false},{&quot;id&quot;:&quot;repository_analytics&quot;,&quot;title&quot;:&quot;Repository analytics&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/graphs/master/charts&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:&quot;shortcuts-repository-charts&quot;,&quot;is_active&quot;:false},{&quot;id&quot;:&quot;code_review&quot;,&quot;title&quot;:&quot;Code review analytics&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/analytics/code_reviews&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:null,&quot;is_active&quot;:false},{&quot;id&quot;:&quot;issues&quot;,&quot;title&quot;:&quot;Issue analytics&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/analytics/issues_analytics&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:null,&quot;is_active&quot;:false},{&quot;id&quot;:&quot;insights&quot;,&quot;title&quot;:&quot;Insights&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/insights/&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:&quot;shortcuts-project-insights&quot;,&quot;is_active&quot;:false},{&quot;id&quot;:&quot;model_experiments&quot;,&quot;title&quot;:&quot;Model experiments&quot;,&quot;icon&quot;:null,&quot;avatar&quot;:null,&quot;entity_id&quot;:null,&quot;link&quot;:&quot;/libeigen/eigen/-/ml/experiments&quot;,&quot;pill_count&quot;:null,&quot;link_classes&quot;:null,&quot;is_active&quot;:false}],&quot;separated&quot;:false}],&quot;current_context_header&quot;:&quot;Project&quot;,&quot;support_path&quot;:&quot;https://about.gitlab.com/get-help/&quot;,&quot;docs_path&quot;:&quot;/help/docs&quot;,&quot;display_whats_new&quot;:true,&quot;whats_new_most_recent_release_items_count&quot;:8,&quot;whats_new_version_digest&quot;:&quot;2b7120854845b6c57c51713a6f7a1efa9f7577a9dc84c00e2224b505a8eddae7&quot;,&quot;show_version_check&quot;:null,&quot;gitlab_version&quot;:{&quot;major&quot;:17,&quot;minor&quot;:1,&quot;patch&quot;:0,&quot;suffix_s&quot;:&quot;&quot;},&quot;gitlab_version_check&quot;:null,&quot;search&quot;:{&quot;search_path&quot;:&quot;/search&quot;,&quot;issues_path&quot;:&quot;/dashboard/issues&quot;,&quot;mr_path&quot;:&quot;/dashboard/merge_requests&quot;,&quot;autocomplete_path&quot;:&quot;/search/autocomplete&quot;,&quot;search_context&quot;:{&quot;group&quot;:{&quot;id&quot;:6577938,&quot;name&quot;:&quot;libeigen&quot;,&quot;full_name&quot;:&quot;libeigen&quot;},&quot;group_metadata&quot;:{&quot;issues_path&quot;:&quot;/groups/libeigen/-/issues&quot;,&quot;mr_path&quot;:&quot;/groups/libeigen/-/merge_requests&quot;},&quot;project&quot;:{&quot;id&quot;:15462818,&quot;name&quot;:&quot;eigen&quot;},&quot;project_metadata&quot;:{&quot;mr_path&quot;:&quot;/libeigen/eigen/-/merge_requests&quot;,&quot;issues_path&quot;:&quot;/libeigen/eigen/-/issues&quot;},&quot;code_search&quot;:true,&quot;ref&quot;:&quot;master&quot;,&quot;scope&quot;:null,&quot;for_snippets&quot;:null}},&quot;panel_type&quot;:&quot;project&quot;,&quot;shortcut_links&quot;:[{&quot;title&quot;:&quot;Snippets&quot;,&quot;href&quot;:&quot;/explore/snippets&quot;,&quot;css_class&quot;:&quot;dashboard-shortcuts-snippets&quot;},{&quot;title&quot;:&quot;Groups&quot;,&quot;href&quot;:&quot;/explore/groups&quot;,&quot;css_class&quot;:&quot;dashboard-shortcuts-groups&quot;},{&quot;title&quot;:&quot;Projects&quot;,&quot;href&quot;:&quot;/explore/projects/starred&quot;,&quot;css_class&quot;:&quot;dashboard-shortcuts-projects&quot;}]}"></aside>

<div class="content-wrapper">

<div class="alert-wrapper gl-force-block-formatting-context">

































<div class="top-bar-fixed container-fluid" data-testid="top-bar">
<div class="top-bar-container gl-display-flex gl-align-items-center gl-gap-2">
<button class="gl-button btn btn-icon btn-md btn-default btn-default-tertiary js-super-sidebar-toggle-expand super-sidebar-toggle -gl-ml-3" aria-controls="super-sidebar" aria-expanded="false" aria-label="Primary navigation sidebar" type="button"><svg class="s16 gl-icon gl-button-icon " data-testid="sidebar-icon"><use href="/assets/icons-0b41337f52be73f7bbf9d59b841eb98a6e790dfa1a844644f120a80ce3cc18ba.svg#sidebar"></use></svg>

</button>
<script type="application/ld+json">
{"@context":"https://schema.org","@type":"BreadcrumbList","itemListElement":[{"@type":"ListItem","position":1,"name":"libeigen","item":"https://gitlab.com/libeigen"},{"@type":"ListItem","position":2,"name":"eigen","item":"https://gitlab.com/libeigen/eigen"},{"@type":"ListItem","position":3,"name":"Repository","item":"https://gitlab.com/libeigen/eigen/-/tree/master"}]}


</script>
<div data-testid="breadcrumb-links" id="js-vue-page-breadcrumbs-wrapper">
<div data-breadcrumbs-json="[{&quot;text&quot;:&quot;libeigen&quot;,&quot;href&quot;:&quot;/libeigen&quot;,&quot;avatarPath&quot;:&quot;/uploads/-/system/group/avatar/6577938/Eigen_Silly_Professor_64x64.png&quot;},{&quot;text&quot;:&quot;eigen&quot;,&quot;href&quot;:&quot;/libeigen/eigen&quot;,&quot;avatarPath&quot;:&quot;/uploads/-/system/project/avatar/15462818/Eigen_Silly_Professor_64x64.png&quot;},{&quot;text&quot;:&quot;Repository&quot;,&quot;href&quot;:&quot;/libeigen/eigen/-/tree/master&quot;,&quot;avatarPath&quot;:null}]" id="js-vue-page-breadcrumbs"></div>
<div id="js-injected-page-breadcrumbs"></div>
</div>



</div>
</div>

</div>
<div class="container-fluid container-limited project-highlight-puc">
<main class="content" id="content-body" itemscope itemtype="http://schema.org/SoftwareSourceCode">
<div class="flash-container flash-container-page sticky" data-testid="flash-container">
<div class="gl-alert flash-notice gl-alert-info" data-testid="alert-info" role="alert">
<div class="gl-alert-icon-container">
<svg class="s16 gl-alert-icon gl-alert-icon-no-title" data-testid="information-o-icon"><use href="/assets/icons-0b41337f52be73f7bbf9d59b841eb98a6e790dfa1a844644f120a80ce3cc18ba.svg#information-o"></use></svg>
</div>
<button class="gl-button btn btn-icon btn-sm btn-default btn-default-tertiary js-close gl-dismiss-btn " aria-label="Dismiss" type="button"><svg class="s16 gl-icon gl-button-icon " data-testid="close-icon"><use href="/assets/icons-0b41337f52be73f7bbf9d59b841eb98a6e790dfa1a844644f120a80ce3cc18ba.svg#close"></use></svg>

</button>
<div class="gl-alert-content" role="alert">
<div class="gl-alert-body">
&quot;debug/gdb/init.py&quot; did not exist on &quot;master&quot;
</div>
</div>
</div>
<div id="js-global-alerts"></div>
</div>







<div class="tree-holder clearfix js-per-page gl-mt-5" data-blame-per-page="1000" id="tree-holder">
<div class="nav-block gl-display-flex gl-flex-direction-column gl-sm-flex-direction-row gl-align-items-stretch">
<div class="tree-ref-container gl-display-flex gl-flex-wrap gl-gap-2 mb-2 mb-md-0">
<div class="tree-ref-holder gl-max-w-26" data-testid="ref-dropdown-container">
<div data-project-id="15462818" data-project-root-path="/libeigen/eigen" data-ref-type="" id="js-tree-ref-switcher"></div>
</div>
<div data-can-collaborate="false" data-can-edit-tree="false" data-can-push-code="false" data-new-blob-path="/libeigen/eigen/-/new/master" data-new-branch-path="/libeigen/eigen/-/branches/new" data-new-dir-path="/libeigen/eigen/-/create_dir/master" data-new-tag-path="/libeigen/eigen/-/tags/new" data-upload-path="/libeigen/eigen/-/create/master" id="js-repo-breadcrumb"></div>
</div>
<div id="js-blob-controls"></div>
<div class="tree-controls">
<div class="gl-display-flex gl-flex-wrap gl-gap-3 gl-mb-3 gl-sm-mb-0">



<div data-history-link="/libeigen/eigen/-/commits/master" id="js-tree-history-link"></div>
<button class="gl-button btn btn-md btn-default has-tooltip shortcuts-find-file" title="Go to file, press &lt;kbd class=&#39;flat ml-1&#39; aria-hidden=true&gt;/~&lt;/kbd&gt; or &lt;kbd class=&#39;flat ml-1&#39; aria-hidden=true&gt;t&lt;/kbd&gt;" aria-keyshortcuts="/+~ t" data-html="true" type="button"><span class="gl-button-text">
Find file

</span>

</button>
<div class="gl-display-inline-block" data-css-classes="gl-w-full gl-sm-w-auto" data-options="{&quot;project_path&quot;:&quot;libeigen/eigen&quot;,&quot;ref&quot;:&quot;master&quot;,&quot;is_fork&quot;:false,&quot;needs_to_fork&quot;:true,&quot;gitpod_enabled&quot;:false,&quot;is_blob&quot;:false,&quot;show_edit_button&quot;:false,&quot;show_web_ide_button&quot;:false,&quot;show_gitpod_button&quot;:false,&quot;show_pipeline_editor_button&quot;:false,&quot;web_ide_url&quot;:&quot;/-/ide/project/libeigen/eigen/edit/master&quot;,&quot;edit_url&quot;:&quot;&quot;,&quot;pipeline_editor_url&quot;:&quot;/libeigen/eigen/-/ci/editor?branch_name=master&quot;,&quot;gitpod_url&quot;:&quot;https://gitpod.io/#https://gitlab.com/libeigen/eigen/-/tree/master/&quot;,&quot;user_preferences_gitpod_path&quot;:&quot;/-/profile/preferences#user_gitpod_enabled&quot;,&quot;user_profile_enable_gitpod_path&quot;:&quot;/-/user_settings/profile?user%5Bgitpod_enabled%5D=true&quot;,&quot;new_workspace_path&quot;:&quot;/-/remote_development/workspaces/new&quot;,&quot;project_id&quot;:15462818,&quot;fork_path&quot;:&quot;/libeigen/eigen/-/forks/new&quot;,&quot;fork_modal_id&quot;:null}" data-web-ide-promo-popover-img="/assets/web-ide-promo-popover-9e59939b3b450a7ea385a520971151abb09ddad46141c333d6dcc783b9b91522.svg" id="js-tree-web-ide-link"></div>

<div class="project-code-holder gl-display-none gl-sm-display-inline-block">
<div class="git-clone-holder js-git-clone-holder">
<div data-directory-download-links="[{&quot;text&quot;:&quot;zip&quot;,&quot;path&quot;:&quot;/libeigen/eigen/-/archive/master/eigen-master.zip&quot;},{&quot;text&quot;:&quot;tar.gz&quot;,&quot;path&quot;:&quot;/libeigen/eigen/-/archive/master/eigen-master.tar.gz&quot;},{&quot;text&quot;:&quot;tar.bz2&quot;,&quot;path&quot;:&quot;/libeigen/eigen/-/archive/master/eigen-master.tar.bz2&quot;},{&quot;text&quot;:&quot;tar&quot;,&quot;path&quot;:&quot;/libeigen/eigen/-/archive/master/eigen-master.tar&quot;}]" data-http-url="https://gitlab.com/libeigen/eigen.git" data-kerberos-url="" data-ssh-url="git@gitlab.com:libeigen/eigen.git" data-xcode-url="" id="js-code-dropdown"></div>
</div>

</div>
</div>
<div class="project-code-holder gl-display-flex gl-gap-3 gl-sm-display-none!">
<div class="js-source-code-dropdown" data-css-class="" data-download-artifacts="[]" data-download-links="[{&quot;text&quot;:&quot;zip&quot;,&quot;path&quot;:&quot;/libeigen/eigen/-/archive/master/eigen-master.zip&quot;},{&quot;text&quot;:&quot;tar.gz&quot;,&quot;path&quot;:&quot;/libeigen/eigen/-/archive/master/eigen-master.tar.gz&quot;},{&quot;text&quot;:&quot;tar.bz2&quot;,&quot;path&quot;:&quot;/libeigen/eigen/-/archive/master/eigen-master.tar.bz2&quot;},{&quot;text&quot;:&quot;tar&quot;,&quot;path&quot;:&quot;/libeigen/eigen/-/archive/master/eigen-master.tar&quot;}]"></div>

<div class="btn-group mobile-git-clone js-mobile-git-clone btn-block">
<button class="gl-button btn btn-md btn-confirm clone-dropdown-btn js-clone-dropdown-label" title="Copy" aria-label="Copy" aria-live="polite" data-toggle="tooltip" data-placement="bottom" data-container="body" data-html="true" data-button-text="Copy HTTPS clone URL" data-size="medium" data-category="primary" data-variant="confirm" data-hide-button-icon="true" data-clipboard-text="https://gitlab.com/libeigen/eigen.git" type="button"><span class="gl-button-text">
Copy HTTPS clone URL
</span>

</button>
<button class="btn gl-button btn-confirm dropdown-toggle js-dropdown-toggle flex-grow-0 d-flex-center w-auto ml-0" data-toggle="dropdown" type="button">
<svg class="s16 dropdown-btn-icon icon" data-testid="chevron-down-icon"><use href="/assets/icons-0b41337f52be73f7bbf9d59b841eb98a6e790dfa1a844644f120a80ce3cc18ba.svg#chevron-down"></use></svg>
</button>
<ul class="dropdown-menu dropdown-menu-selectable dropdown-menu-right clone-options-dropdown" data-dropdown>
<li class="js-clone-links">
<a class="copy ssh clone url-selector is-active" href="git@gitlab.com:libeigen/eigen.git" data-clone-type="ssh"><strong class="dropdown-menu-inner-title">Copy SSH clone URL</strong><span class="dropdown-menu-inner-content">git@gitlab.com:libeigen/eigen.git</span></a>
</li>
<li class="js-clone-links">
<a class="copy https clone url-selector " href="https://gitlab.com/libeigen/eigen.git" data-clone-type="http"><strong class="dropdown-menu-inner-title">Copy HTTPS clone URL</strong><span class="dropdown-menu-inner-content">https://gitlab.com/libeigen/eigen.git</span></a>
</li>

</ul>
</div>

</div>
</div>

</div>
<div class="info-well gl-display-none gl-sm-display-flex project-last-commit gl-flex-direction-column gl-mt-5">
<div class="gl-m-auto" data-ref-type="" id="js-last-commit">
<div class="gl-spinner-container" role="status"><span aria-label="Loading" class="gl-spinner gl-spinner-md gl-spinner-dark gl-vertical-align-text-bottom!"></span></div>
</div>
<div data-branch="master" data-branch-rules-path="/libeigen/eigen/-/settings/repository#js-branch-rules" id="js-code-owners"></div>
</div>
<div data-escaped-ref="master" data-explain-code-available="false" data-full-name="libeigen / eigen" data-path-locks-available="true" data-path-locks-toggle="/libeigen/eigen/path_locks/toggle" data-project-path="libeigen/eigen" data-project-short-path="eigen" data-ref="master" data-resource-id="gid://gitlab/Project/15462818" data-user-id="" id="js-tree-list"></div>
</div>

<script nonce="P8NEvu5zQj6vpUmdeGqmoA==">
//<![CDATA[
  window.gl = window.gl || {};
  window.gl.webIDEPath = '/-/ide/project/libeigen/eigen/edit/master'


//]]>
</script>
<div data-ambiguous="false" data-ref="master" id="js-ambiguous-ref-modal"></div>

</main>
</div>


</div>
</div>


<script nonce="P8NEvu5zQj6vpUmdeGqmoA==">
//<![CDATA[
if ('loading' in HTMLImageElement.prototype) {
  document.querySelectorAll('img.lazy').forEach(img => {
    img.loading = 'lazy';
    let imgUrl = img.dataset.src;
    // Only adding width + height for avatars for now
    if (imgUrl.indexOf('/avatar/') > -1 && imgUrl.indexOf('?') === -1) {
      const targetWidth = img.getAttribute('width') || img.width;
      imgUrl += `?width=${targetWidth}`;
    }
    img.src = imgUrl;
    img.removeAttribute('data-src');
    img.classList.remove('lazy');
    img.classList.add('js-lazy-loaded');
    img.dataset.testid = 'js-lazy-loaded-content';
  });
}

//]]>
</script>
<script nonce="P8NEvu5zQj6vpUmdeGqmoA==">
//<![CDATA[
gl = window.gl || {};
gl.experiments = {};


//]]>
</script>

</body>
</html>

