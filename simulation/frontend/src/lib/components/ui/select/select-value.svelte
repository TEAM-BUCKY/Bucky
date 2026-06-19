<script lang="ts">
	import { Select as SelectPrimitive } from "bits-ui";

	let { ...restProps }: SelectPrimitive.ValueProps = $props();
</script>

<SelectPrimitive.Value data-slot="select-value" {...restProps} />
